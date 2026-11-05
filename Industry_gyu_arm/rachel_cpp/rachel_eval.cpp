// RACHEL C++ 추론기 — 의존성 없는 단일 실행파일.
//   목적 ①: 라떼판다에서 torch 없이 돌리기 (2.5GB -> 20MB)
//   목적 ②: 이 코드가 곧 8086/Z80 어셈블리의 설계도이자 정답 대조표
//
//   빌드:  g++ -O3 -march=native -o rachel rachel.cpp
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
// 단일 스레드가 정답이다 — 모델이 작아(320차원) OpenMP 스레드 생성비가
// 계산보다 크다. 실측: 단일 168ms / OpenMP 617~761ms (4배 느림).
static void matvec(const Tensor& W, const float* x, float* y, int rows, int cols) {
    const float sc = W.scale;
    const int8_t* base = W.q.data();
    for (int i = 0; i < rows; i++) {
        const int8_t* w = base + (size_t)i*cols;
        float a0=0, a1=0, a2=0, a3=0;          // 4중 누산 — 파이프라인 채우기
        int j = 0;
        for (; j+3 < cols; j += 4) {
            a0 += (float)w[j]   * x[j];
            a1 += (float)w[j+1] * x[j+1];
            a2 += (float)w[j+2] * x[j+2];
            a3 += (float)w[j+3] * x[j+3];
        }
        float acc = a0+a1+a2+a3;
        for (; j < cols; j++) acc += (float)w[j] * x[j];
        y[i] = acc * sc;
    }
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
    vector<float> x, h, q_, k_, v_, att, mlp;
    Runner(Model& mm) : m(mm), kv(mm.layers) {
        int d = m.dim;
        for (auto& c : kv) { c.k.resize((size_t)m.ctx*d); c.v.resize((size_t)m.ctx*d); }
        x.resize(d); h.resize(d); q_.resize(d); k_.resize(d); v_.resize(d);
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
            // q,k,v 를 한 번에 (in_proj 는 3d 행)
            {
                const int8_t* Wb = Wq.q.data(); const float qs = Wq.scale;
                float* qp=q_.data(); float* kp=k_.data(); float* vp=v_.data();
                const float* hp = h.data();
                for (int r = 0; r < 3*d; r++) {
                    const int8_t* w = Wb + (size_t)r*d;
                    float a0=0,a1=0,a2=0,a3=0; int j=0;
                    for (; j+3 < d; j += 4) {
                        a0 += (float)w[j]  *hp[j];   a1 += (float)w[j+1]*hp[j+1];
                        a2 += (float)w[j+2]*hp[j+2]; a3 += (float)w[j+3]*hp[j+3];
                    }
                    float val = a0+a1+a2+a3;
                    for (; j < d; j++) val += (float)w[j]*hp[j];
                    val *= qs;
                    if (Bq.n) val += Bq.q[r]*Bq.scale;
                    if (r < d) qp[r] = val;
                    else if (r < 2*d) kp[r-d] = val;
                    else vp[r-2*d] = val;
                }
            }
            KV& c = kv[l];
            memcpy(&c.k[(size_t)pos*d], k_.data(), d*sizeof(float));
            memcpy(&c.v[(size_t)pos*d], v_.data(), d*sizeof(float));
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

int main(int argc, char** argv) {
    if (argc < 2) { printf("사용: %s rachel.bin [문장|--bench]\n", argv[0]); return 1; }
    Model m;
    auto t0 = chrono::steady_clock::now();
    if (!load(argv[1], m)) return 1;
    auto t1 = chrono::steady_clock::now();
    map<string,int> stoi;
    for (size_t i=0;i<m.itos.size();i++) stoi[m.itos[i]] = i;
    fprintf(stderr, "d%d L%d H%d ctx%d vocab%d · 로드 %.0fms\n",
            m.dim, m.layers, m.heads, m.ctx, m.vocab,
            chrono::duration<double,milli>(t1-t0).count());

    // ── 상주 모드 (--serve) ────────────────────────────────────
    //   모델을 한 번만 읽고 표준입력으로 명령을 계속 받는다.
    //   매번 새로 실행하면 프로세스 시작 + 9.6MB 로드가 붙어 100ms 손해다.
    //   입력: 한 줄에 프롬프트 전체 ('\n' 대신 '\x01' 로 줄바꿈 표현)
    //   출력: 결과 한 줄
    if (argc > 2 && !strcmp(argv[2], "--serve")) {
        Runner sr(m);
        vector<float> lg;
        string line;
        printf("READY\n"); fflush(stdout);
        while (getline(cin, line)) {
            if (line == "QUIT") break;
            for (auto& c : line) if (c == '\x01') c = '\n';
            sr.reset(); string res;
            auto chars = utf8_chars(line);
            int pos = 0;
            for (auto& ch : chars) {
                auto it = stoi.find(ch);
                sr.step(it == stoi.end() ? 1 : it->second, pos++, lg);
            }
            for (int n = 0; n < 400 && pos < 4*m.ctx; n++) {
                int bi = 0; float bv = -1e30f;
                for (int i = 0; i < m.vocab; i++) if (lg[i] > bv) { bv = lg[i]; bi = i; }
                if (bi == 2) break;
                if (bi >= 3) res += m.itos[bi];
                sr.step(bi, pos++, lg);
            }
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
        r.reset(); out.clear();
        auto ts = chrono::steady_clock::now();
        auto chars = utf8_chars(prompt);
        int pos = 0, tok = 0;
        for (auto& ch : chars) {                    // 프롬프트 주입
            auto it = stoi.find(ch);
            tok = (it==stoi.end()) ? 1 : it->second;
            r.step(tok, pos++, logits);
        }
        // 프롬프트 마지막 step 의 logits 가 곧 '첫 글자' 다.
        // 이걸 버리고 생성 루프에 들어가면 한 글자가 통째로 빠진다 ({" 누락).
        for (int n = 0; n < 400 && pos < 4*m.ctx; n++) {   // 생성
            int best_i = 0; float best_v = -1e30f;
            for (int i=0;i<m.vocab;i++) if (logits[i] > best_v) { best_v=logits[i]; best_i=i; }
            if (best_i == 2) break;                 // EOS
            if (best_i >= 3) out += m.itos[best_i];
            r.step(best_i, pos++, logits);
        }
        double ms = chrono::duration<double,milli>(chrono::steady_clock::now()-ts).count();
        if (ms < best) best = ms;
        if (bench) fprintf(stderr, "  회차 %d: %.0fms\n", rep+1, ms);
    }
    printf("%s\n", out.c_str());
    fprintf(stderr, "생성 %.0fms (최고)\n", best);
    return 0;
}
