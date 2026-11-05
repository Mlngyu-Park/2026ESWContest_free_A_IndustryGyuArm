// RACHEL C++ 추론기 — 의존성 없는 단일 실행파일.
//   목적 ①: 라떼판다에서 torch 없이 돌리기 (2.5GB -> 20MB)
//   목적 ②: 이 코드가 곧 8086/Z80 어셈블리의 설계도이자 정답 대조표
//
//   빌드:  g++ -O3 -march=native -ffast-math -pthread -o rachel rachel.cpp
//          (★ -ffast-math 꼭 — 없으면 4배 느리다: 0.27초 -> 1.17초, 2026-10-29 실측)
//          라떼판다 등 옮길 때: g++ -O3 -ffast-math -pthread -static -o rachel_static rachel.cpp
//   실행:  ./rachel rachel.bin "장면\n힌트\n명령"
//   벤치:  ./rachel rachel.bin --bench
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
using namespace std;

// ── 가중치 파일 ───────────────────────────────────────────────
struct Tensor { vector<int8_t> q; float scale; int n; };
struct Model {
    int dim, layers, heads, ctx, vocab;
    vector<string> itos;
    map<string, Tensor> t;
    int head_dim() const { return dim / heads; }
};

static bool load(const char* path, Model& m) {
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "파일 없음: %s\n", path); return false; }
    char magic[4]; uint32_t ver;
    fread(magic, 1, 4, f); fread(&ver, 4, 1, f);
    if (memcmp(magic, "RCHL", 4)) { fprintf(stderr, "형식 아님\n"); return false; }
    uint32_t d, L, h, c, v;
    fread(&d,4,1,f); fread(&L,4,1,f); fread(&h,4,1,f); fread(&c,4,1,f); fread(&v,4,1,f);
    m.dim=d; m.layers=L; m.heads=h; m.ctx=c; m.vocab=v;
    uint32_t vlen; fread(&vlen,4,1,f);
    string vb(vlen, 0); fread(&vb[0], 1, vlen, f);
    // 어휘: [길이 u8][UTF-8 바이트] 반복. 줄바꿈 구분은 '\n' 글자 자체가
    // 어휘에 들어 있어 한 칸씩 밀리는 사고가 났다.
    for (size_t p = 0; p < vb.size(); ) {
        int n = (unsigned char)vb[p++];
        m.itos.push_back(vb.substr(p, n)); p += n;
    }
    uint32_t nt; fread(&nt,4,1,f);
    for (uint32_t i = 0; i < nt; i++) {
        uint16_t nl; fread(&nl,2,1,f);
        string nm(nl,0); fread(&nm[0],1,nl,f);
        Tensor tt; uint32_t cnt; fread(&cnt,4,1,f); fread(&tt.scale,4,1,f);
        tt.n = cnt; tt.q.resize(cnt);
        fread(tt.q.data(), 1, cnt, f);
        m.t[nm] = move(tt);
    }
    fclose(f); return true;
}

// ── int8 행렬-벡터 곱 (이 루프가 8086 MAC 커널이 된다) ─────────
//   y[i] = scale * sum_j( W[i][j] * x[j] )
//   가중치만 int8, 활성값은 float 로 둔다 (W8A32).
//   활성값까지 int8 로 누르면 값이 ±1 을 넘는 순간 잘려 출력이 무너진다
//   — 실제로 그렇게 짰다가 "겨겨겨…" 만 나왔다.
//   8086 커널에서는 활성값도 별도 스케일로 int16 화한다 (여기선 참조 구현).
// ── 여러 코어로 나눠 계산 (2026-10-31) ────────────────────────
//   예전: 단일 스레드 — OpenMP 는 곱셈마다 스레드를 깨워서 오히려 4배 느렸다
//         (단일 168ms / OpenMP 617~761ms, 320차원 v9).
//   지금: 일꾼 스레드를 처음에 한 번 띄워 두고, 행렬 곱셈의 '행' 을 나눠 맡긴다.
//         일꾼은 곱셈 사이에는 잠깐 돌며 기다리고(깨우는 비용 없음),
//         명령을 기다리는 동안에는 잠들어 CPU 를 쓰지 않는다.
//         행 하나는 늘 한 스레드가 예전과 같은 순서로 더하므로 결과가 단일 스레드와 같다.
//   스레드 수: RACHEL_THREADS (기본 = CPU 스레드의 절반, 노트북 16→8 · N305 8→4)
static inline float dot_row(const int8_t* w, const float* x, int cols) {
    float a0=0, a1=0, a2=0, a3=0;              // 4중 누산 — 파이프라인 채우기
    int j = 0;
    for (; j+3 < cols; j += 4) {
        a0 += (float)w[j]   * x[j];
        a1 += (float)w[j+1] * x[j+1];
        a2 += (float)w[j+2] * x[j+2];
        a3 += (float)w[j+3] * x[j+3];
    }
    float acc = a0+a1+a2+a3;
    for (; j < cols; j++) acc += (float)w[j] * x[j];
    return acc;
}

// y[r] = W[r]·x * scale (+ bias[r])  — 행 [r0, r1) 만
struct MvJob { const Tensor* W; const Tensor* B; const float* x; float* y; int rows, cols; };
static void mv_rows(const MvJob& jb, int r0, int r1) {
    const float sc = jb.W->scale;
    const int8_t* base = jb.W->q.data();
    const bool hasb = jb.B && jb.B->n;
    for (int i = r0; i < r1; i++) {
        float val = dot_row(base + (size_t)i*jb.cols, jb.x, jb.cols) * sc;
        if (hasb) val += jb.B->q[i] * jb.B->scale;
        jb.y[i] = val;
    }
}

static inline void cpu_relax() {
#if defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#endif
}

struct Pool {
    int n = 1;                                  // 계산하는 스레드 수 (main 포함)
    vector<thread> ws;
    atomic<uint64_t> gen{0};                    // 일감 번호 — 바뀌면 일꾼이 일어난다
    atomic<int> left{0}, sleepers{0};
    atomic<bool> quit{false};
    mutex mu; condition_variable cv;
    MvJob job{};

    void chunk(int id) {
        int r0 = (int)((long)job.rows * id / n), r1 = (int)((long)job.rows * (id + 1) / n);
        mv_rows(job, r0, r1);
    }
    void worker(int id) {
        uint64_t seen = 0;
        for (;;) {
            int spins = 0;
            while (gen.load() == seen && !quit.load()) {
                if (++spins < 200000) { cpu_relax(); continue; }   // 곱셈 사이: 잠깐 돌며 기다림
                unique_lock<mutex> lk(mu);                          // 오래 쉬면: 잠든다
                sleepers++;
                cv.wait(lk, [&] { return gen.load() != seen || quit.load(); });
                sleepers--;
                spins = 0;
            }
            if (quit.load()) return;
            seen = gen.load();
            chunk(id);
            left.fetch_sub(1);
        }
    }
    void start(int threads) {
        n = max(1, threads);
        for (int i = 1; i < n; i++) ws.emplace_back(&Pool::worker, this, i);
    }
    void run(const MvJob& jb) {
        // 작은 곱셈은 나누는 비용이 더 크다 → 혼자 한다
        if (n == 1 || (long)jb.rows * jb.cols < 65536) { mv_rows(jb, 0, jb.rows); return; }
        job = jb;
        left.store(n - 1);
        gen.fetch_add(1);
        if (sleepers.load() > 0) { lock_guard<mutex> lk(mu); cv.notify_all(); }
        chunk(0);
        while (left.load() > 0) cpu_relax();
    }
    ~Pool() {
        quit.store(true);
        { lock_guard<mutex> lk(mu); cv.notify_all(); }
        for (auto& t : ws) t.join();
    }
};
static Pool pool;

static int threads_() {
    const char* e = getenv("RACHEL_THREADS");
    if (e && atoi(e) > 0) return atoi(e);
    int hw = (int)thread::hardware_concurrency();
    return max(1, hw / 2);
}

static void matvec(const Tensor& W, const float* x, float* y, int rows, int cols) {
    pool.run(MvJob{&W, nullptr, x, y, rows, cols});
}
static void addvec(float* y, const Tensor& b, int n) {
    if (!b.n) return;
    for (int i = 0; i < n; i++) y[i] += b.q[i] * b.scale;
}
static void layernorm(float* x, const Tensor& g, const Tensor& b, int n) {
    float mu = 0; for (int i=0;i<n;i++) mu += x[i]; mu /= n;
    float va = 0; for (int i=0;i<n;i++) { float d=x[i]-mu; va += d*d; } va = va/n + 1e-5f;
    float inv = 1.0f / sqrtf(va);
    for (int i=0;i<n;i++) {
        float gv = g.n ? g.q[i]*g.scale : 1.0f;
        float bv = b.n ? b.q[i]*b.scale : 0.0f;
        x[i] = (x[i]-mu)*inv*gv + bv;
    }
}
static void gelu(float* x, int n) {
    for (int i=0;i<n;i++) {
        float v=x[i];
        x[i] = 0.5f*v*(1.0f + tanhf(0.7978845608f*(v + 0.044715f*v*v*v)));
    }
}

// ── KV 캐시 — 이게 속도의 핵심 ────────────────────────────────
struct KV { vector<float> k, v; int len = 0; };

struct Runner {
    Model& m;
    vector<KV> kv;
    vector<float> x, h, qkv, att, mlp;
    Runner(Model& mm) : m(mm), kv(mm.layers) {
        int d = m.dim;
        for (auto& c : kv) { c.k.resize((size_t)m.ctx*d); c.v.resize((size_t)m.ctx*d); }
        x.resize(d); h.resize(d); qkv.resize(3*d);
        att.resize(m.ctx); mlp.resize(4*d);
    }
    void reset() { for (auto& c : kv) c.len = 0; }

    const Tensor& T(const string& s) {
        auto it = m.t.find(s);
        static Tensor empty{{},1,0};
        return it == m.t.end() ? empty : it->second;
    }

    // 한 토큰 전방 계산 -> logits (vocab)
    void step(int tok, int pos, vector<float>& logits) {
        int d = m.dim, H = m.heads, hd = m.head_dim();
        const Tensor& emb = T("tok.weight");
        for (int i=0;i<d;i++) x[i] = emb.q[(size_t)tok*d+i] * emb.scale;
        const Tensor& pe = T("pos.weight");
        for (int i=0;i<d;i++) x[i] += pe.q[(size_t)pos*d+i] * pe.scale;

        for (int l = 0; l < m.layers; l++) {
            string p = "blocks." + to_string(l) + ".";
            // ── 어텐션
            memcpy(h.data(), x.data(), d*sizeof(float));
            layernorm(h.data(), T(p+"ln1.weight"), T(p+"ln1.bias"), d);
            const Tensor& Wq = T(p+"attn.in_proj_weight");   // [3d, d]
            const Tensor& Bq = T(p+"attn.in_proj_bias");
            // q,k,v 를 한 번에 (in_proj 는 3d 행) → qkv = [q | k | v]
            pool.run(MvJob{&Wq, &Bq, h.data(), qkv.data(), 3*d, d});
            const float* q_ = qkv.data();
            KV& c = kv[l];
            memcpy(&c.k[(size_t)pos*d], qkv.data() + d, d*sizeof(float));
            memcpy(&c.v[(size_t)pos*d], qkv.data() + 2*d, d*sizeof(float));
            c.len = pos+1;
            // 헤드별 어텐션
            for (int i=0;i<d;i++) h[i]=0;
            for (int hh = 0; hh < H; hh++) {
                float inv = 1.0f/sqrtf((float)hd), mx = -1e30f;
                for (int t2 = 0; t2 <= pos; t2++) {
                    float s = 0;
                    for (int j = 0; j < hd; j++)
                        s += q_[hh*hd+j] * c.k[(size_t)t2*d + hh*hd + j];
                    att[t2] = s*inv; if (att[t2] > mx) mx = att[t2];
                }
                float sum = 0;
                for (int t2=0;t2<=pos;t2++) { att[t2]=expf(att[t2]-mx); sum+=att[t2]; }
                for (int t2=0;t2<=pos;t2++) {
                    float a = att[t2]/sum;
                    for (int j=0;j<hd;j++) h[hh*hd+j] += a * c.v[(size_t)t2*d + hh*hd + j];
                }
            }
            // 출력 투영
            static vector<float> o; o.resize(d);
            matvec(T(p+"attn.out_proj.weight"), h.data(), o.data(), d, d);
            addvec(o.data(), T(p+"attn.out_proj.bias"), d);
            for (int i=0;i<d;i++) x[i] += o[i];
            // ── MLP
            memcpy(h.data(), x.data(), d*sizeof(float));
            layernorm(h.data(), T(p+"ln2.weight"), T(p+"ln2.bias"), d);
            matvec(T(p+"mlp.0.weight"), h.data(), mlp.data(), 4*d, d);
            addvec(mlp.data(), T(p+"mlp.0.bias"), 4*d);
            gelu(mlp.data(), 4*d);
            matvec(T(p+"mlp.2.weight"), mlp.data(), o.data(), d, 4*d);
            addvec(o.data(), T(p+"mlp.2.bias"), d);
            for (int i=0;i<d;i++) x[i] += o[i];
        }
        layernorm(x.data(), T("ln.weight"), T("ln.bias"), d);
        logits.resize(m.vocab);
        matvec(T("tok.weight"), x.data(), logits.data(), m.vocab, d);  // 가중치 공유
    }
};

// ── UTF-8 <-> 글자 인덱스 ────────────────────────────────────
static vector<string> utf8_chars(const string& s) {
    vector<string> out;
    for (size_t i = 0; i < s.size(); ) {
        unsigned char c = s[i];
        int n = c<0x80?1 : (c>>5)==6?2 : (c>>4)==14?3 : 4;
        out.push_back(s.substr(i, n)); i += n;
    }
    return out;
}

// ── 생성 — 출력 한도와 문맥 넘김 (2026-10-29) ─────────────────
//   예전: 최대 200글자, 문맥(448)이 차면 멈춤 → 5단계 답(약 270자)이 잘렸고,
//         입력이 문맥보다 길면 KV 캐시 밖에 썼다 (메모리 오류).
//   지금: 최대 RACHEL_MAXGEN(기본 400)글자 (5단계 답 236자·6단계 약 280자), 답이 닫히면(']}') 바로 멈춤. 문맥이 차면 '장면·힌트 줄' 은 그대로 두고
//         그 뒤 가장 오래된 부분을 밀어내며 이어 쓴다 (장면 줄이 있어야 도킹 번호를 바로 쓴다).
static int max_gen() {
    const char* e = getenv("RACHEL_MAXGEN");
    int v = e ? atoi(e) : 400;
    return v > 0 ? v : 400;
}
// 온도 뽑기 (더블 레이첼 다수결용, 2026-10-30). RACHEL_TEMP 가 없거나 0 이면 지금처럼 가장 큰 값.
static float temp_() { const char* e = getenv("RACHEL_TEMP"); return e ? (float)atof(e) : 0.0f; }
static uint64_t rng_ = 0x9E3779B97F4A7C15ULL;
static float urand() { rng_ ^= rng_ << 13; rng_ ^= rng_ >> 7; rng_ ^= rng_ << 17; return (rng_ >> 40) / 16777216.0f; }
static int pick(const vector<float>& lg, int V, float T) {
    int bi = 0; float bv = -1e30f;
    for (int i = 0; i < V; i++) if (lg[i] > bv) { bv = lg[i]; bi = i; }
    if (T <= 0) return bi;
    static vector<float> p; p.resize(V); float s = 0;
    for (int i = 0; i < V; i++) { p[i] = expf((lg[i] - bv) / T); s += p[i]; }
    float u = urand() * s;
    for (int i = 0; i < V; i++) { u -= p[i]; if (u <= 0) return i; }
    return bi;
}

static string generate(Model& m, Runner& r, map<string,int>& stoi, const string& prompt) {
    const int SLACK = 64;                       // 한 번 밀어낼 때 비워 두는 칸 (매 글자마다 다시 계산하지 않게)
    vector<int> seq;
    for (auto& ch : utf8_chars(prompt)) {
        auto it = stoi.find(ch);
        seq.push_back(it == stoi.end() ? 1 : it->second);
    }
    // 머리 = 마지막 문장 줄 앞까지 (장면 줄 + 힌트 줄). 줄바꿈이 두 개 이상일 때만
    int nl = stoi.count("\n") ? stoi["\n"] : -1, hdr = 0;
    {
        vector<int> nls;
        for (size_t i = 0; i < seq.size(); i++) if (seq[i] == nl) nls.push_back((int)i);
        if (nls.size() >= 2) hdr = nls[nls.size() - 2] + 1;
    }
    const int budget = m.ctx - 1;
    if (hdr > budget / 2) hdr = 0;              // 머리가 지나치게 길면 지키지 않는다
    auto fit = [&](const vector<int>& v) {
        if ((int)v.size() <= budget) return v;
        int keep = budget - hdr - SLACK;
        vector<int> o(v.begin(), v.begin() + hdr);
        o.insert(o.end(), v.end() - keep, v.end());
        return o;
    };
    vector<float> lg;
    vector<int> cur = fit(seq);
    r.reset();
    for (size_t i = 0; i < cur.size(); i++) r.step(cur[i], (int)i, lg);
    string out;
    int lim = max_gen(); float T = temp_();
    for (int n = 0; n < lim; n++) {
        int bi = pick(lg, m.vocab, T);
        if (bi == 2) break;                     // EOS
        if (bi >= 3) out += m.itos[bi];
        // 답이 닫혔으면 멈춘다 — 모델이 끝 표시를 빠뜨리고 같은 말을 되풀이해도 오래 돌지 않게
        if (out.size() >= 2 && out.compare(out.size() - 2, 2, "]}") == 0) break;
        cur.push_back(bi);
        if ((int)cur.size() > budget) {         // 문맥이 찼다 → 밀어내고 다시 계산
            cur = fit(cur);
            r.reset();
            for (size_t i = 0; i < cur.size(); i++) r.step(cur[i], (int)i, lg);
        } else {
            r.step(bi, (int)cur.size() - 1, lg);
        }
    }
    return out;
}

int main(int argc, char** argv) {
    if (argc < 2) { printf("사용: %s rachel.bin [문장|--bench]\n", argv[0]); return 1; }
    Model m;
    auto t0 = chrono::steady_clock::now();
    if (!load(argv[1], m)) return 1;
    auto t1 = chrono::steady_clock::now();
    map<string,int> stoi;
    for (size_t i=0;i<m.itos.size();i++) stoi[m.itos[i]] = i;
    pool.start(threads_());
    fprintf(stderr, "d%d L%d H%d ctx%d vocab%d · 로드 %.0fms · 스레드 %d\n",
            m.dim, m.layers, m.heads, m.ctx, m.vocab,
            chrono::duration<double,milli>(t1-t0).count(), pool.n);

    // ── 상주 모드 (--serve) ────────────────────────────────────
    //   모델을 한 번만 읽고 표준입력으로 명령을 계속 받는다.
    //   매번 새로 실행하면 프로세스 시작 + 9.6MB 로드가 붙어 100ms 손해다.
    //   입력: 한 줄에 프롬프트 전체 ('\n' 대신 '\x01' 로 줄바꿈 표현)
    //   출력: 결과 한 줄
    if (getenv("RACHEL_SEED")) rng_ ^= (uint64_t)atoll(getenv("RACHEL_SEED")) * 0x2545F4914F6CDD1DULL;
    if (argc > 2 && !strcmp(argv[2], "--serve")) {
        Runner sr(m);
        vector<float> lg;
        string line;
        printf("READY\n"); fflush(stdout);
        while (getline(cin, line)) {
            if (line == "QUIT") break;
            for (auto& c : line) if (c == '\x01') c = '\n';
            string res = generate(m, sr, stoi, line);
            for (auto& c : res) if (c == '\n') c = '\x01';   // 답 안의 줄바꿈 (더블 레이첼: 정리문\n=블록) → 한 줄로
            printf("%s\n", res.c_str()); fflush(stdout);
        }
        return 0;
    }

    string prompt = (argc > 2 && strcmp(argv[2],"--bench"))
        ? argv[2]
        : "빨강=d8 노랑=d12 주황=d9 | 펜=초록@d8\n힌트 d8 d12\n"
          "빨간 도킹의 초록 펜을 노란 데로 옮겨\n>";   // '>' 가 생성 시작 신호
    bool bench = (argc > 2 && !strcmp(argv[2],"--bench"));
    int reps = bench ? 3 : 1;

    Runner r(m);
    vector<float> logits;
    if (getenv("DEBUG_ONE")) {                 // 토큰 하나만 넣고 logits 확인
        r.step(atoi(getenv("DEBUG_ONE")), 0, logits);
        vector<pair<float,int>> v;
        for (int i=0;i<m.vocab;i++) v.push_back({logits[i],i});
        sort(v.rbegin(), v.rend());
        for (int i=0;i<3;i++)
            fprintf(stderr, "  %3d '%s' %.3f\n", v[i].second,
                    m.itos[v[i].second].c_str(), v[i].first);
        return 0;
    }
    double best = 1e18;
    string out;
    for (int rep = 0; rep < reps; rep++) {
        auto ts = chrono::steady_clock::now();
        // 프롬프트 마지막 step 의 logits 가 곧 '첫 글자' 다 (generate 안에서 이어 쓴다)
        out = generate(m, r, stoi, prompt);
        double ms = chrono::duration<double,milli>(chrono::steady_clock::now()-ts).count();
        if (ms < best) best = ms;
        if (bench) fprintf(stderr, "  회차 %d: %.0fms\n", rep+1, ms);
    }
    printf("%s\n", out.c_str());
    fprintf(stderr, "생성 %.0fms (최고)\n", best);
    return 0;
}
