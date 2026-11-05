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
    void (*fn)(void*, int, int) = nullptr;      // 행렬 곱 말고 다른 일 (어텐션 헤드 나누기)
    void* fctx = nullptr;

    void chunk(int id) {
        if (fn) { fn(fctx, id, n); return; }
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
    void run_fn(void (*f)(void*, int, int), void* c) {
        if (n == 1) { f(c, 0, 1); return; }
        fn = f; fctx = c;
        left.store(n - 1);
        gen.fetch_add(1);
        if (sleepers.load() > 0) { lock_guard<mutex> lk(mu); cv.notify_all(); }
        chunk(0);
        while (left.load() > 0) cpu_relax();
        fn = nullptr;
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

// ── DUSK_BOOM TPU (2026-11-05) ────────────────────────────────────────────────────────────────────
//   RACHEL_TPU=cpu (기본)  예전과 같다 (W8A32 float)
//   RACHEL_TPU=emu         TPU 와 똑같은 정수 계산을 CPU 로 (정확도 확인용)
//   RACHEL_TPU=hw          보드 FPGA 의 TPU (tpu_hw.h, /dev/mem)
//   활성값 x 는 곱셈마다 따로 int16 화: sx = max|x| / XQ, xi = round(x / sx), |xi| <= XQ
//   acc = sum W[i][j] * xi[j] (정수, int8 x int16), y = acc * W.scale * sx (+ bias)
//   XQ = 8191 -> |acc| <= 1792 * 127 * 8191 < 2^31 : TPU 의 32 비트 누산이 넘칠 수 없다
static const int XQ = 8191;
static int tpu_mode() {
    static int m = -1;
    if (m < 0) {
        const char* e = getenv("RACHEL_TPU");
        m = (e && !strcmp(e, "emu")) ? 1 : (e && !strcmp(e, "hw")) ? 2 : 0;
    }
    return m;
}
static float quant16(const float* x, int n, int16_t* xi) {
    // 반올림은 0 에서 먼 쪽 (lrintf 는 보드에서 느린 함수 호출이라 바꿈, 2026-11-05)
    float mx = 0;
    for (int i = 0; i < n; i++) mx = fmaxf(mx, fabsf(x[i]));
    if (mx == 0) { memset(xi, 0, n * sizeof(int16_t)); return 0.f; }
    float sx = mx / XQ, inv = 1.0f / sx;
    const float lim = (float)XQ;
    for (int i = 0; i < n; i++) {
        float v = fminf(fmaxf(x[i] * inv, -lim), lim);
        xi[i] = (int16_t)(int)(v + (v >= 0.f ? 0.5f : -0.5f));
    }
    return sx;
}
static void mv_emu(const Tensor& W, const Tensor* B, const float* x, float* y, int rows, int cols) {
    static vector<int16_t> xi; xi.resize(cols);
    float sx = quant16(x, cols, xi.data());
    const int8_t* w = W.q.data();
    const float k = W.scale * sx;
    const bool hasb = B && B->n;
    for (int i = 0; i < rows; i++) {
        int32_t acc = 0;
        const int8_t* r = w + (size_t)i * cols;
        for (int j = 0; j < cols; j++) acc += (int32_t)r[j] * xi[j];
        y[i] = acc * k + (hasb ? B->q[i] * B->scale : 0.f);
    }
}
// ── TPU 화면용 기록 (2026-11-05, RACHEL_TRACE=1) ─────────────────────────────────────────────
//   표준오류로 한 줄씩: #S 프롬프트글자수 · #M 층 종류 행 열 벡터 마이크로초 클럭0 클럭1 · #P (프롬프트 끝)
//   · #T 글자 · #E 걸린ms. 보드 화면(tpu_show.py)이 읽어 곱셈기·층 그림을 움직인다.
static int TR_L = -1;
static const char* TR_K = "";
// 누적값 (#S 에서 0) — 화면 프로그램이 따라오도록 20ms 에 한 줄만 쓴다 (#C 층 종류 행 열 벡터 횟수 마이크로초 바이트 곱셈)
static long TR_calls = 0;
static double TR_us = 0, TR_bytes = 0, TR_macs = 0, TR_last = 0;
static bool trace_on() { static int v = -1; if (v < 0) v = getenv("RACHEL_TRACE") ? 1 : 0; return v; }
#ifdef DUSK_TPU_HW
#include "tpu_hw.h"
#endif
static void mv(const Tensor& W, const Tensor* B, const float* x, float* y, int rows, int cols) {
    switch (tpu_mode()) {
    case 1: mv_emu(W, B, x, y, rows, cols); return;
#ifdef DUSK_TPU_HW
    case 2: if (tpu_hw_mv(W, B, x, y, rows, cols)) return; break;   // 못 하면 CPU 로
#endif
    default: break;
    }
    pool.run(MvJob{&W, B, x, y, rows, cols});
}
// 벡터 n 개 (n <= 4) 를 같은 가중치로 — TPU 는 가중치를 한 번만 읽는다 (프롬프트 묶음 처리)
static void mv_n(const Tensor& W, const Tensor* B, const float* const* xs, float* const* ys, int n, int rows, int cols) {
#ifdef DUSK_TPU_HW
    if (tpu_mode() == 2 && tpu_hw_mvn(W, B, xs, ys, n, rows, cols)) return;
#endif
    for (int t = 0; t < n; t++) mv(W, B, xs[t], ys[t], rows, cols);
}
static void matvec(const Tensor& W, const float* x, float* y, int rows, int cols) {
    mv(W, nullptr, x, y, rows, cols);
}
static double prof_att = 0, prof_gelu = 0, prof_head = 0;
static inline double secs() { return chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count(); }
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
// 빠른 tanh / exp (2026-11-05, 보드 Cortex-A9 에서 tanhf·expf 가 함수 호출이라 GELU 1.25 초·어텐션 대부분을 먹었다)
//   tanh: 유리식 근사 (Eigen ptanh_float 계수, float 오차 몇 ulp), exp: Cephes expf 다항식.
//   둘 다 if 없는 산술이라 컴파일러가 NEON 으로 묶는다. RACHEL_SLOWMATH=1 이면 예전 함수.
static inline float fast_tanh(float x) {
    const float c = 7.90531110763549805f;
    x = fminf(fmaxf(x, -c), c);
    const float x2 = x * x;
    float p = -2.76076847742355e-16f;
    p = p * x2 + 2.00018790482477e-13f;
    p = p * x2 - 8.60467152213735e-11f;
    p = p * x2 + 5.12229709037114e-08f;
    p = p * x2 + 1.48572235717979e-05f;
    p = p * x2 + 6.37261928875436e-04f;
    p = p * x2 + 4.89352455891786e-03f;
    p = p * x;
    float q = 1.19825839466702e-06f;
    q = q * x2 + 1.18534705686654e-04f;
    q = q * x2 + 2.26843463243900e-03f;
    q = q * x2 + 4.89352518554385e-03f;
    return p / q;
}
static inline float fast_exp(float x) {
    x = fminf(fmaxf(x, -87.0f), 88.0f);
    float fn = x * 1.44269504088896341f;
    fn = (float)(int)(fn + (fn >= 0.f ? 0.5f : -0.5f));
    float r = x - fn * 0.693359375f;
    r = r + fn * 2.12194440e-4f;
    float p = 1.9875691500E-4f;
    p = p * r + 1.3981999507E-3f;
    p = p * r + 8.3334519073E-3f;
    p = p * r + 4.1665795894E-2f;
    p = p * r + 1.6666665459E-1f;
    p = p * r + 5.0000001201E-1f;
    float y = p * r * r + r + 1.0f;
    int32_t e = ((int32_t)fn + 127) << 23;
    float s; memcpy(&s, &e, 4);
    return y * s;
}
static bool slowmath() { static int v = -1; if (v < 0) v = getenv("RACHEL_SLOWMATH") ? 1 : 0; return v; }
static void gelu_part(float* x, int n);
struct GeluJob { float* x; int n; };
static void gelu_fn(void* p, int id, int k) {
    GeluJob& j = *(GeluJob*)p;
    int a = (int)((long)j.n * id / k) & ~3, b = id + 1 == k ? j.n : (int)((long)j.n * (id + 1) / k) & ~3;
    gelu_part(j.x + a, b - a);
}
static void gelu(float* x, int n) {
    GeluJob j{x, n};
    if (pool.n > 1 && pool.n <= 8 && n >= 1024) pool.run_fn(gelu_fn, &j);
    else gelu_part(x, n);
}
static void gelu_part(float* x, int n) {
    if (slowmath()) {
        for (int i=0;i<n;i++) {
            float v=x[i];
            x[i] = 0.5f*v*(1.0f + tanhf(0.7978845608f*(v + 0.044715f*v*v*v)));
        }
        return;
    }
    for (int i=0;i<n;i++) {
        float v=x[i];
        x[i] = 0.5f*v*(1.0f + fast_tanh(0.7978845608f*(v + 0.044715f*v*v*v)));
    }
}

// ── KV 캐시 — 이게 속도의 핵심 ────────────────────────────────
//   배치 (2026-11-05): 헤드마다 따로 [헤드][위치][hd] — 한 헤드가 읽는 K·V 가 메모리에 이어져 있어
//   보드에서 미리읽기가 잘 듣는다 (예전 [위치][d] 는 줄마다 224 바이트씩 건너뛰며 읽었다). 계산은 같다.
//   KV16 (보드 빌드 기본): K·V 를 fp16 으로 저장 — 어텐션은 메모리 읽기에 걸려 있어 읽는 양이 절반이 된다.
#if defined(DUSK_TPU_HW) && !defined(KV32)
#define KV16 1
#endif
#ifdef KV16
#if defined(__ARM_NEON) && !defined(__aarch64__)
typedef __fp16 kv_t;
#else
#include <immintrin.h>                       // 노트북 확인용: F16C 로 같은 반올림
struct kv_t {
    uint16_t b;
    kv_t() = default;
    kv_t(float f) : b((uint16_t)_cvtss_sh(f, 0)) {}
    operator float() const { return _cvtsh_ss(b); }
};
#endif
#else
typedef float kv_t;
#endif
struct KV {
    vector<kv_t> k, v; int len = 0; int ctx = 0, hd = 0, H = 0;
    size_t at(int hh, int t) const { return ((size_t)hh * ctx + t) * hd; }
    void put(int pos, const float* kk, const float* vv) {
        for (int hh = 0; hh < H; hh++)
            for (int j = 0; j < hd; j++) { k[at(hh, pos) + j] = (kv_t)kk[hh*hd + j]; v[at(hh, pos) + j] = (kv_t)vv[hh*hd + j]; }
    }
};

#if defined(__ARM_NEON) && !defined(__aarch64__)
#include <arm_neon.h>
#ifdef KV16
#define LDK(p) vcvt_f32_f16(vreinterpret_f16_u16(vld1_u16((const uint16_t*)(p))))
#else
#define LDK(p) vld1q_f32(p)
#endif
// 미리읽기: 한 줄(hd 개) 이 32 바이트 캐시 라인 여러 개라 라인마다 한 번씩, 약 512 바이트 앞을 읽어 둔다
#define PF(p) do { const char* pc_ = (const char*)(p) + 512; \
    for (int o_ = 0; o_ < hd * (int)sizeof(kv_t); o_ += 32) __builtin_prefetch(pc_ + o_); } while (0)
// 보드(Cortex-A9) 전용 NEON 어텐션 (2026-11-05 다시 짬)
//   A9 는 NEON 과 VFP(스칼라 실수) 를 번갈아 쓰면 매번 수십 클럭 멈춘다 — 줄마다 점수를 스칼라로 꺼내던
//   예전 방식이 한 줄에 530 클럭이었다. 지금은 4 줄씩 묶어 점수·최대값·exp·합·가중합을 모두 NEON 안에서.
//   스칼라로 꺼내는 건 헤드마다 두 번 (최대값, 합) 뿐.
static inline float32x4_t exp_v(float32x4_t x) {           // fast_exp 의 NEON 판 (같은 식)
    x = vminq_f32(vmaxq_f32(x, vdupq_n_f32(-87.0f)), vdupq_n_f32(88.0f));
    float32x4_t fn = vmulq_n_f32(x, 1.44269504088896341f);
    const uint32x4_t ge = vcgeq_f32(fn, vdupq_n_f32(0.f));
    fn = vaddq_f32(fn, vbslq_f32(ge, vdupq_n_f32(0.5f), vdupq_n_f32(-0.5f)));
    const int32x4_t ni = vcvtq_s32_f32(fn);
    fn = vcvtq_f32_s32(ni);
    float32x4_t r = vmlsq_f32(x, fn, vdupq_n_f32(0.693359375f));
    r = vmlaq_f32(r, fn, vdupq_n_f32(2.12194440e-4f));
    float32x4_t p = vdupq_n_f32(1.9875691500E-4f);
    p = vmlaq_f32(vdupq_n_f32(1.3981999507E-3f), p, r);
    p = vmlaq_f32(vdupq_n_f32(8.3334519073E-3f), p, r);
    p = vmlaq_f32(vdupq_n_f32(4.1665795894E-2f), p, r);
    p = vmlaq_f32(vdupq_n_f32(1.6666665459E-1f), p, r);
    p = vmlaq_f32(vdupq_n_f32(5.0000001201E-1f), p, r);
    float32x4_t y = vaddq_f32(vmlaq_f32(r, p, vmulq_f32(r, r)), vdupq_n_f32(1.0f));
    const int32x4_t e = vshlq_n_s32(vaddq_s32(ni, vdupq_n_s32(127)), 23);
    return vmulq_f32(y, vreinterpretq_f32_s32(e));
}
static inline float32x2_t hsum2(float32x4_t a) { return vpadd_f32(vget_low_f32(a), vget_high_f32(a)); }
static void attend_heads_neon(const float* q_, int pos, const KV& c, float* h, float* att, int d, int hd, int h0, int h1) {
    (void)d;
    const float inv = 1.0f/sqrtf((float)hd);
    const int n = pos + 1, n4 = (n + 3) & ~3;
    const int rowb = hd * (int)sizeof(kv_t);
    for (int hh = h0; hh < h1; hh++) {
        const float* q = q_ + hh*hd;
        const kv_t* K = c.k.data() + c.at(hh, 0);
        const kv_t* V = c.v.data() + c.at(hh, 0);
        // 1) 점수: 4 줄씩
        int t2 = 0;
        for (; t2 + 4 <= n; t2 += 4) {
            const kv_t* k0 = K + (size_t)t2*hd;
            const char* pf = (const char*)k0 + 512;
            for (int o = 0; o < 4*rowb; o += 32) __builtin_prefetch(pf + o);
            float32x4_t s0 = vdupq_n_f32(0), s1 = s0, s2 = s0, s3 = s0;
            for (int j = 0; j < hd; j += 4) {
                const float32x4_t qv = vld1q_f32(q + j);
                s0 = vmlaq_f32(s0, qv, LDK(k0 + j));
                s1 = vmlaq_f32(s1, qv, LDK(k0 + hd + j));
                s2 = vmlaq_f32(s2, qv, LDK(k0 + 2*hd + j));
                s3 = vmlaq_f32(s3, qv, LDK(k0 + 3*hd + j));
            }
            const float32x4_t r = vcombine_f32(vpadd_f32(hsum2(s0), hsum2(s1)), vpadd_f32(hsum2(s2), hsum2(s3)));
            vst1q_f32(att + t2, vmulq_n_f32(r, inv));
        }
        for (; t2 < n; t2++) {
            const kv_t* k0 = K + (size_t)t2*hd;
            float32x4_t s0 = vdupq_n_f32(0);
            for (int j = 0; j < hd; j += 4) s0 = vmlaq_f32(s0, vld1q_f32(q + j), LDK(k0 + j));
            float32x2_t p = hsum2(s0);
            vst1_lane_f32(att + t2, vmul_n_f32(vpadd_f32(p, p), inv), 0);
        }
        for (int i = n; i < n4; i++) ((uint32_t*)att)[i] = 0xF149F2CAu;   // -1e30 (정수로 써서 VFP 를 안 거침)
        // 2) 최대값
        float32x4_t mv = vld1q_f32(att);
        for (int i = 4; i < n4; i += 4) mv = vmaxq_f32(mv, vld1q_f32(att + i));
        float32x2_t m2 = vpmax_f32(vget_low_f32(mv), vget_high_f32(mv));
        m2 = vpmax_f32(m2, m2);
        const float32x4_t mx = vcombine_f32(m2, m2);
        // 3) exp 와 합
        float32x4_t sv = vdupq_n_f32(0);
        for (int i = 0; i < n4; i += 4) {
            float32x4_t e = exp_v(vsubq_f32(vld1q_f32(att + i), mx));
            vst1q_f32(att + i, e);
        }
        for (int i = n; i < n4; i++) ((uint32_t*)att)[i] = 0;               // 패딩은 0
        for (int i = 0; i < n4; i += 4) sv = vaddq_f32(sv, vld1q_f32(att + i));
        float32x2_t s2 = hsum2(sv);
        s2 = vpadd_f32(s2, s2);
        const float is = 1.0f / vget_lane_f32(s2, 0);                         // 헤드마다 한 번
        for (int i = 0; i < n4; i += 4) vst1q_f32(att + i, vmulq_n_f32(vld1q_f32(att + i), is));
        // 4) 가중합: 16 칸씩 누산을 레지스터에
        float* o = h + hh*hd;
        for (int j0 = 0; j0 < hd; j0 += 16) {
            const int w = hd - j0 >= 16 ? 16 : hd - j0;
            float32x4_t a0 = vdupq_n_f32(0), a1 = a0, a2 = a0, a3 = a0;
            for (int t = 0; t < n; t++) {
                const kv_t* v = V + (size_t)t*hd + j0;
                if (j0 == 0) { const char* pf = (const char*)v + 512; for (int o2 = 0; o2 < rowb; o2 += 32) __builtin_prefetch(pf + o2); }
                const float32x4_t a = vld1q_dup_f32(att + t);
                a0 = vmlaq_f32(a0, a, LDK(v));
                if (w > 4)  a1 = vmlaq_f32(a1, a, LDK(v + 4));
                if (w > 8)  a2 = vmlaq_f32(a2, a, LDK(v + 8));
                if (w > 12) a3 = vmlaq_f32(a3, a, LDK(v + 12));
            }
            vst1q_f32(o + j0, a0);
            if (w > 4)  vst1q_f32(o + j0 + 4, a1);
            if (w > 8)  vst1q_f32(o + j0 + 8, a2);
            if (w > 12) vst1q_f32(o + j0 + 12, a3);
        }
    }
}
#endif
// 어텐션: 헤드 [h0, h1) — 위치 pos 의 q 가 0..pos 를 본다. 헤드마다 계산 순서는 예전과 같다.
static void attend_heads(const float* q_, int pos, const KV& c, float* h, float* att, int d, int hd, int h0, int h1) {
    for (int hh = h0; hh < h1; hh++) {
        for (int j = 0; j < hd; j++) h[hh*hd+j] = 0;
        float inv = 1.0f/sqrtf((float)hd), mx = -1e30f;
        for (int t2 = 0; t2 <= pos; t2++) {
            float s = 0;
            for (int j = 0; j < hd; j++)
                s += q_[hh*hd+j] * (float)c.k[c.at(hh, t2) + j];
            att[t2] = s*inv; if (att[t2] > mx) mx = att[t2];
        }
        float sum = 0;
        if (slowmath()) for (int t2=0;t2<=pos;t2++) att[t2]=expf(att[t2]-mx);
        else            for (int t2=0;t2<=pos;t2++) att[t2]=fast_exp(att[t2]-mx);
        for (int t2=0;t2<=pos;t2++) sum+=att[t2];
        for (int t2=0;t2<=pos;t2++) {
            float a = att[t2]/sum;
            for (int j=0;j<hd;j++) h[hh*hd+j] += a * (float)c.v[c.at(hh, t2) + j];
        }
    }
}
struct AttJob { const float* q; int pos; const KV* c; float* h; int d, hd, H, ctx; };
static vector<float> att_buf[8];
static void att_fn(void* p, int id, int n) {
    AttJob& j = *(AttJob*)p;
    att_buf[id].resize(j.ctx + 8);
#if defined(__ARM_NEON) && !defined(__aarch64__)
    if (!slowmath() && (j.hd & 3) == 0) {
        attend_heads_neon(j.q, j.pos, *j.c, j.h, att_buf[id].data(), j.d, j.hd, j.H * id / n, j.H * (id + 1) / n);
        return;
    }
#endif
    attend_heads(j.q, j.pos, *j.c, j.h, att_buf[id].data(), j.d, j.hd, j.H * id / n, j.H * (id + 1) / n);
}
static void attend(const float* q, int pos, const KV& c, float* h, int d, int hd, int H, int ctx) {
    double t0 = secs();
    AttJob j{q, pos, &c, h, d, hd, H, ctx};
    if (pool.n > 1 && pool.n <= 8 && pos >= 16) pool.run_fn(att_fn, &j);
    else att_fn(&j, 0, 1);
    prof_att += secs() - t0;
}

struct Runner {
    Model& m;
    vector<KV> kv;
    vector<float> x, h, qkv, att, mlp;
    Runner(Model& mm) : m(mm), kv(mm.layers) {
        int d = m.dim;
        for (auto& c : kv) {
            c.k.resize((size_t)m.ctx*d); c.v.resize((size_t)m.ctx*d);
            c.ctx = m.ctx; c.hd = m.head_dim(); c.H = m.heads;
        }
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
    void step(int tok, int pos, vector<float>& logits, bool want = true) {
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
            TR_L = l; TR_K = "qkv";
            mv(Wq, &Bq, h.data(), qkv.data(), 3*d, d);
            const float* q_ = qkv.data();
            KV& c = kv[l];
            c.put(pos, qkv.data() + d, qkv.data() + 2*d);
            c.len = pos+1;
            // 헤드별 어텐션
            attend(q_, pos, c, h.data(), d, hd, H, m.ctx);
            // 출력 투영
            static vector<float> o; o.resize(d);
            TR_K = "out";
            matvec(T(p+"attn.out_proj.weight"), h.data(), o.data(), d, d);
            addvec(o.data(), T(p+"attn.out_proj.bias"), d);
            for (int i=0;i<d;i++) x[i] += o[i];
            // ── MLP
            memcpy(h.data(), x.data(), d*sizeof(float));
            layernorm(h.data(), T(p+"ln2.weight"), T(p+"ln2.bias"), d);
            TR_K = "up";
            matvec(T(p+"mlp.0.weight"), h.data(), mlp.data(), 4*d, d);
            addvec(mlp.data(), T(p+"mlp.0.bias"), 4*d);
            { double tg = secs(); gelu(mlp.data(), 4*d); prof_gelu += secs() - tg; }
            TR_K = "down";
            matvec(T(p+"mlp.2.weight"), mlp.data(), o.data(), d, 4*d);
            addvec(o.data(), T(p+"mlp.2.bias"), d);
            for (int i=0;i<d;i++) x[i] += o[i];
        }
        if (!want) return;                       // 프롬프트 중간 글자: 다음 글자 점수가 필요 없다
        double th = secs();
        layernorm(x.data(), T("ln.weight"), T("ln.bias"), d);
        logits.resize(m.vocab);
        TR_L = m.layers; TR_K = "head";
        matvec(T("tok.weight"), x.data(), logits.data(), m.vocab, d);  // 가중치 공유
        prof_head += secs() - th;
    }

    // 토큰 n 개 (n <= 4) 를 한 번에 — 프롬프트용. 토큰마다의 계산은 step() 과 똑같다
    // (행렬 곱만 묶어서 가중치를 한 번 읽는다). 마지막 토큰의 logits 만 낸다.
    vector<float> bx[4], bh[4], bq[4], bm[4], bo[4];
    void step_n(const int* toks, int n, int pos0, vector<float>& logits, bool want) {
        if (n == 1) { step(toks[0], pos0, logits, want); return; }
        int d = m.dim, H = m.heads, hd = m.head_dim();
        const Tensor& emb = T("tok.weight");
        const Tensor& pe = T("pos.weight");
        const float *ph[4], *pq[4], *pm[4]; float *wh[4], *wq[4], *wm[4], *wo[4];
        for (int t = 0; t < n; t++) {
            bx[t].resize(d); bh[t].resize(d); bq[t].resize(3*d); bm[t].resize(4*d); bo[t].resize(d);
            for (int i=0;i<d;i++) bx[t][i] = emb.q[(size_t)toks[t]*d+i] * emb.scale;
            for (int i=0;i<d;i++) bx[t][i] += pe.q[(size_t)(pos0+t)*d+i] * pe.scale;
            ph[t] = wh[t] = bh[t].data(); pq[t] = wq[t] = bq[t].data(); pm[t] = wm[t] = bm[t].data(); wo[t] = bo[t].data();
        }
        for (int l = 0; l < m.layers; l++) {
            string p = "blocks." + to_string(l) + ".";
            for (int t = 0; t < n; t++) {
                memcpy(bh[t].data(), bx[t].data(), d*sizeof(float));
                layernorm(bh[t].data(), T(p+"ln1.weight"), T(p+"ln1.bias"), d);
            }
            const Tensor& Bq = T(p+"attn.in_proj_bias");
            TR_L = l; TR_K = "qkv";
            mv_n(T(p+"attn.in_proj_weight"), &Bq, ph, wq, n, 3*d, d);
            KV& c = kv[l];
            for (int t = 0; t < n; t++) {
                c.put(pos0+t, bq[t].data() + d, bq[t].data() + 2*d);
            }
            c.len = pos0+n;
            for (int t = 0; t < n; t++) attend(bq[t].data(), pos0+t, c, bh[t].data(), d, hd, H, m.ctx);
            TR_K = "out";
            mv_n(T(p+"attn.out_proj.weight"), nullptr, ph, wo, n, d, d);
            for (int t = 0; t < n; t++) {
                addvec(bo[t].data(), T(p+"attn.out_proj.bias"), d);
                for (int i=0;i<d;i++) bx[t][i] += bo[t][i];
                memcpy(bh[t].data(), bx[t].data(), d*sizeof(float));
                layernorm(bh[t].data(), T(p+"ln2.weight"), T(p+"ln2.bias"), d);
            }
            TR_K = "up";
            mv_n(T(p+"mlp.0.weight"), nullptr, ph, wm, n, 4*d, d);
            for (int t = 0; t < n; t++) {
                addvec(bm[t].data(), T(p+"mlp.0.bias"), 4*d);
                double tg = secs(); gelu(bm[t].data(), 4*d); prof_gelu += secs() - tg;
            }
            TR_K = "down";
            mv_n(T(p+"mlp.2.weight"), nullptr, pm, wo, n, d, 4*d);
            for (int t = 0; t < n; t++) {
                addvec(bo[t].data(), T(p+"mlp.2.bias"), d);
                for (int i=0;i<d;i++) bx[t][i] += bo[t][i];
            }
        }
        if (!want) return;
        double th = secs();
        float* xl = bx[n-1].data();
        layernorm(xl, T("ln.weight"), T("ln.bias"), d);
        logits.resize(m.vocab);
        TR_L = m.layers; TR_K = "head";
        matvec(T("tok.weight"), xl, logits.data(), m.vocab, d);
        prof_head += secs() - th;
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

static int batch_() {
    static int b = -1;
    if (b < 0) { const char* e = getenv("RACHEL_BATCH"); b = e ? atoi(e) : 4; if (b < 1) b = 1; if (b > 4) b = 4; }
    return b;
}
// 프롬프트 넣기: 토큰을 batch 개씩 묶어서, logits 는 맨 마지막 토큰만
static void prefill(Runner& r, const vector<int>& cur, vector<float>& lg) {
    r.reset();
    const int B = batch_(), N = (int)cur.size();
    for (int i = 0; i < N; ) {
        int n = min(B, N - i);
        r.step_n(&cur[i], n, i, lg, i + n == N);
        i += n;
    }
}
static string generate_(Model& m, Runner& r, map<string,int>& stoi, const string& prompt);
static string generate(Model& m, Runner& r, map<string,int>& stoi, const string& prompt) {
#ifdef DUSK_TPU_HW
    if (tpu_mode() == 2) {                      // 답 하나 동안 TPU 를 혼자 쓴다 (다른 rachel 프로세스와 나눠 씀)
        tpu::dev.begin();
        string o = generate_(m, r, stoi, prompt);
        tpu::dev.end();
        return o;
    }
#endif
    return generate_(m, r, stoi, prompt);
}
static string generate_(Model& m, Runner& r, map<string,int>& stoi, const string& prompt) {
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
    const auto g0 = chrono::steady_clock::now();
    auto el_ms = [&] { return chrono::duration<double, milli>(chrono::steady_clock::now() - g0).count(); };
    TR_calls = 0; TR_us = TR_bytes = TR_macs = 0; TR_last = 0;
    if (trace_on()) { fprintf(stderr, "#S %d\n", (int)cur.size()); fflush(stderr); }
    prefill(r, cur, lg);
    if (trace_on()) { fprintf(stderr, "#P %.0f\n", el_ms()); fflush(stderr); }
    // 속도 맞춤 (RACHEL_PACE_MS): 화면용으로 계산을 그 시간에 고르게 편다. 글자 수는 RACHEL_PACE_EST(기본 70)로 어림
    const char* pe = getenv("RACHEL_PACE_MS");
    const double pace = pe ? atof(pe) : 0.0;
    const double pace_est = getenv("RACHEL_PACE_EST") ? atof(getenv("RACHEL_PACE_EST")) : 70.0;
    const double t_pre = el_ms();
    string out;
    int lim = max_gen(); float T = temp_();
    for (int n = 0; n < lim; n++) {
        int bi = pick(lg, m.vocab, T);
        if (bi == 2) break;                     // EOS
        if (bi >= 3) out += m.itos[bi];
        if (trace_on() && bi >= 3) {
            string tk = m.itos[bi];
            if (tk == "\n") tk = "\\n";
            fprintf(stderr, "#T %s\n", tk.c_str()); fflush(stderr);
        }
        if (pace > 0) {
            const double want = t_pre + (pace - t_pre) * (n + 1) / pace_est;
            const double now = el_ms();
            if (want > now) this_thread::sleep_for(chrono::microseconds((long)((want - now) * 1000)));
        }
        // 답이 닫혔으면 멈춘다 — 모델이 끝 표시를 빠뜨리고 같은 말을 되풀이해도 오래 돌지 않게
        if (out.size() >= 2 && out.compare(out.size() - 2, 2, "]}") == 0) break;
        cur.push_back(bi);
        if ((int)cur.size() > budget) {         // 문맥이 찼다 → 밀어내고 다시 계산
            cur = fit(cur);
            prefill(r, cur, lg);
        } else {
            r.step(bi, (int)cur.size() - 1, lg);
        }
    }
    if (trace_on()) {
        fprintf(stderr, "#C %d %s 0 0 1 %ld %.0f %.0f %.0f\n", TR_L, TR_K, TR_calls, TR_us, TR_bytes, TR_macs);
        fprintf(stderr, "#E %.0f\n", el_ms()); fflush(stderr);
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
    fprintf(stderr, "gen_ms=%.0f\n", best);
    if (getenv("RACHEL_PROF"))
        fprintf(stderr, "[prof] attention %.0f  gelu %.0f  head %.0f ms (all reps)\n", prof_att*1000, prof_gelu*1000, prof_head*1000);
#ifdef DUSK_TPU_HW
    if (tpu_mode() == 2 && tpu::dev.ok)
        fprintf(stderr, "[tpu] %ld 번 곱셈, TPU 기다린 시간 %.0fms\n", tpu::dev.calls, tpu::dev.t_hw * 1000);
    if (tpu_mode() == 2 && tpu::dev.ok)
        fprintf(stderr, "[tpu] quant %.0f  x-copy %.0f  regs %.0f  y-read %.0f ms\n", tpu::dev.t_q * 1000,
                tpu::dev.t_put * 1000, tpu::dev.t_reg * 1000, tpu::dev.t_rd * 1000);
#endif
    return 0;
}
