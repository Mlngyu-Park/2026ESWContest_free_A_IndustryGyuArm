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
/* ── 단계적 정수화 (68000 이식 검증용) ────────────────────
   정답(float)에서 출발해 한 단계씩 정수로 바꾸며 어디서 깨지는지 본다.
   여기서 100% 를 유지하는 조합이 곧 68000 에 옮길 정답이다. */
static int st(const char* k){ return getenv(k)!=NULL; }
static uint32_t isqrt32(uint32_t v){
    if(!v) return 0; uint32_t x=v,y=(x+1)>>1;
    while(y<x){x=y;y=(x+v/x)>>1;} return x;
}

static void matvec(const Tensor& W, const float* x, float* y, int rows, int cols) {
    const float sc = W.scale;
    const int8_t* base = W.q.data();
    if (st("STEP_MV")) {
        float mx=0; for(int j=0;j<cols;j++){float a=fabsf(x[j]); if(a>mx)mx=a;}
        if(mx==0) mx=1;
        float xs=mx/32767.0f;
        static vector<int16_t> xq; xq.resize(cols);
        for(int j=0;j<cols;j++) xq[j]=(int16_t)lrintf(x[j]/xs);
        for(int i=0;i<rows;i++){
            const int8_t* w=base+(size_t)i*cols;
            int32_t acc=0;
            for(int j=0;j<cols;j++) acc+=(int32_t)w[j]*(int32_t)xq[j];
            y[i]=acc*sc*xs;
        }
        return;
    }
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
    if (st("STEP_LN")) {
        /* 대조용 실수 결과 (LN_CMP 일 때만) */
        static vector<float> ref;
        if (st("LN_CMP")) {
            ref.assign(x,x+n);
            float mu=0; for(int i=0;i<n;i++) mu+=ref[i]; mu/=n;
            float va=0; for(int i=0;i<n;i++){float d=ref[i]-mu; va+=d*d;} va=va/n+1e-5f;
            float iv=1.0f/sqrtf(va);
            for(int i=0;i<n;i++){
                float gv=g.n?g.q[i]*g.scale:1.0f, bv=b.n?b.q[i]*b.scale:0.0f;
                ref[i]=(ref[i]-mu)*iv*gv+bv;
            }
        }
        /* 68000 방식: 활성값 int16, 정수 연산만 */
        float mx=0; for(int i=0;i<n;i++){float a=fabsf(x[i]); if(a>mx)mx=a;}
        if(mx==0) mx=1;
        float xs=mx/32767.0f;
        static vector<int16_t> q; q.resize(n);
        for(int i=0;i<n;i++) q[i]=(int16_t)lrintf(x[i]/xs);
        int32_t sum=0; for(int i=0;i<n;i++) sum+=q[i];
        int32_t mean=sum/n;
        int32_t var=0;
        for(int i=0;i<n;i++){int32_t d=q[i]-mean; var+=(d*d)>>8;}
        var/=n;
        int32_t sd=(int32_t)isqrt32((uint32_t)var); if(sd<1) sd=1;
        int nsh = getenv("LN_SH")?atoi(getenv("LN_SH")):11;
        for(int i=0;i<n;i++){
            int32_t d=q[i]-mean;
            int64_t y=((int64_t)d<<nsh)/sd;
            /* ★ 실제 gamma = g.q[i] * g.scale 이다. scale 안에 int8 을
                 실수로 되돌리는 몫이 이미 들어 있으므로 >>7 을 또 하면
                 128 로 나눠진다 (실측: 정수판이 실수판의 1/8, gamma 항만
                 보면 118배 작았다). q16 스케일이니 >>16 만 한다. */
            /* 시프트 20 은 실측으로 정했다:
                 >>23 -> 정수판이 실수판의 1/8
                 >>16 -> 16배 (포화)
                 >>20 -> 1.0배  ← 맞다
               (nsh=11 의 정규화 배율과 sd 의 q11 단위가 합쳐진 결과다) */
            if(g.n) y=(y*g.q[i]*(int32_t)lrintf(g.scale*65536))>>20;
            if(b.n) y+=((int64_t)b.q[i]*(int32_t)lrintf(b.scale*65536)*(1<<nsh))>>20;
            if(y>32767) y=32767; if(y<-32768) y=-32768;
            x[i]=(float)y/(float)(1<<nsh);
            if(st("LN_CMP") && i<6)
                fprintf(stderr,"  [%d] 실수 %+.5f · 정수 %+.5f · 차 %+.5f\n",
                        i, ref[i], x[i], x[i]-ref[i]);
        }
        return;
    }
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
    if (st("STEP_GL")) {
        /* 68000 방식: 512칸 표 + 선형보간, 범위 ±4.0
           (표 읽기 8T vs 곱셈 70T — 표가 더 싸고 더 정확하다) */
        for(int i=0;i<n;i++){
            float v=x[i];
            float t=v/4.0f*256.0f;
            int k=(int)floorf(t); float f=t-k;
            if(k<-256){k=-256;f=0;} if(k>254){k=254;f=1;}
            float x0=k*4.0f/256, x1=(k+1)*4.0f/256;
            float g0=0.5f*x0*(1+tanhf(0.7978845608f*(x0+0.044715f*x0*x0*x0)));
            float g1=0.5f*x1*(1+tanhf(0.7978845608f*(x1+0.044715f*x1*x1*x1)));
            x[i]=g0+(g1-g0)*f;
        }
        return;
    }
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
              if (st("STEP_AT")) {
                /* 68000 방식 어텐션 — 가장 단순한 형태로.
                   q,k 를 int16 으로 놓고 int32 로 누산한다 (MULS + ADD.L).
                   softmax 입력은 실수로 되돌려 계산한다 (여기서는 exp 를
                   정수화하지 않는다 — 한 번에 하나씩 확인하기 위해). */
                float qmx=0, kmx=0;
                for(int j=0;j<hd;j++){float a=fabsf(q_[hh*hd+j]); if(a>qmx)qmx=a;}
                for(int t2=0;t2<=pos;t2++)
                    for(int j=0;j<hd;j++){
                        float a=fabsf(c.k[(size_t)t2*d+hh*hd+j]); if(a>kmx)kmx=a; }
                if(qmx==0)qmx=1; if(kmx==0)kmx=1;
                /* ★ 폭이 int32 누산 한계로 정해진다 — 실측 버그:
                     +-32000 이면 hd=40 개 곱의 합이 409억 -> int32(21억) 넘침.
                     실제로 -933,560,989 (부호 뒤집힘) 이 나와 점수가
                     +1.32 -> -0.37 로 망가졌다.
                     안전 한계 sqrt(2^31/40) = +-7327. +-5000 으로 여유를 둔다.
                     68000 의 MULS(16x16->32) + ADD.L(32비트) 제약 그대로다. */
                float qs=qmx/5000.0f, ks=kmx/5000.0f;

                static vector<int16_t> qq; qq.resize(hd);
                for(int j=0;j<hd;j++) qq[j]=(int16_t)lrintf(q_[hh*hd+j]/qs);

                static vector<float> att; att.resize(pos+1);
                float amx=-1e30f;
                for(int t2=0;t2<=pos;t2++){
                    int32_t acc=0;
                    for(int j=0;j<hd;j++){
                        int16_t kq=(int16_t)lrintf(c.k[(size_t)t2*d+hh*hd+j]/ks);
                        acc += (int32_t)qq[j]*(int32_t)kq;
                    }
                    att[t2] = acc*qs*ks/sqrtf((float)hd);
                    if(att[t2]>amx) amx=att[t2];
                }
                float asum=0;
                for(int t2=0;t2<=pos;t2++){ att[t2]=expf(att[t2]-amx); asum+=att[t2]; }
                if (st("CMP")) {
                    /* 같은 실행에서 실수 경로도 계산해 비교 */
                    static vector<float> fa; fa.resize(pos+1);
                    float fmx=-1e30f;
                    for(int t2=0;t2<=pos;t2++){
                        float sc=0;
                        for(int j=0;j<hd;j++) sc+=q_[hh*hd+j]*c.k[(size_t)t2*d+hh*hd+j];
                        fa[t2]=sc/sqrtf((float)hd); if(fa[t2]>fmx)fmx=fa[t2];
                    }
                    float fs=0;
                    for(int t2=0;t2<=pos;t2++){fa[t2]=expf(fa[t2]-fmx); fs+=fa[t2];}
                    double maxd=0;
                    for(int t2=0;t2<=pos;t2++){
                        double dd=fabs(fa[t2]/fs - att[t2]/asum);
                        if(dd>maxd) maxd=dd;
                    }
                    if (maxd > 0.01 && pos<=2 && l==0 && hh==2) {
                        fprintf(stderr,"  L%d h%d pos%d 확률차 %.4f\n",l,hh,pos,maxd);
                        for(int t2=0;t2<=pos;t2++){
                            /* 원시 점수까지 비교 */
                            float fsc=0;
                            for(int j=0;j<hd;j++) fsc+=q_[hh*hd+j]*c.k[(size_t)t2*d+hh*hd+j];
                            int32_t acc=0; int64_t acc64=0;
                            for(int j=0;j<hd;j++){
                                int32_t kq=(int32_t)lrintf(c.k[(size_t)t2*d+hh*hd+j]/ks);
                                acc+=(int32_t)qq[j]*kq;
                                acc64+=(int64_t)qq[j]*kq;
                            }
                            if (acc != acc64)
                                fprintf(stderr,"    ★ int32 누산 넘침! %d vs %lld\n",
                                        acc, (long long)acc64);
                            /* 이 위치 k 의 크기와 양자화 결과 */
                            float km=0; int nsat=0;
                            for(int j=0;j<hd;j++){
                                float a=fabsf(c.k[(size_t)t2*d+hh*hd+j]); if(a>km)km=a;
                                int32_t v=(int32_t)lrintf(c.k[(size_t)t2*d+hh*hd+j]/ks);
                                if(v>32767||v<-32768) nsat++;
                            }
                            fprintf(stderr,"    t%d 점수 실수 %+.6f · 정수 %+.6f | "
                                    "k최대 %.4f (ks기준 %.0f) 넘침 %d개\n",
                                    t2, fsc/sqrtf((float)hd), acc*qs*ks/sqrtf((float)hd),
                                    km, km/ks, nsat);
                        }
                    }
                }
                for(int t2=0;t2<=pos;t2++){
                    float a=att[t2]/asum;
                    for(int j=0;j<hd;j++)
                        h[hh*hd+j] += a * c.v[(size_t)t2*d+hh*hd+j];
                }
                continue;
              }
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
            for (int n = 0; n < 200 && pos < m.ctx; n++) {
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
        for (int n = 0; n < 200 && pos < m.ctx; n++) {   // 생성
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
