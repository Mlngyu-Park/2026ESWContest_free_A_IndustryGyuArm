/* ============================================================
 *  DUSK — RACHEL 추론 (정수 전용)
 *
 *  자작 언어모델 RACHEL(10.1M) 을 68000 에서 돌리기 위한 코드다.
 *  rachel_cpp/rachel.cpp 와 같은 계산을 하되, float 를 쓰지 않는다.
 *
 *  왜 정수여야 하나
 *    68000 에는 FPU 가 없다. float 곱셈 하나를 소프트웨어로 하면
 *    수백 사이클이 든다. int16 곱셈(MULS)은 70 사이클이다.
 *
 *  고정소수점 규약 (dusk.h 와 같음)
 *    q15_t  활성값. 1.0 = 32767
 *    q31_t  누산기. MULS 결과를 32비트로 모은다
 *
 *  검증 방법
 *    --check 를 주면 float 로도 같이 계산해 두 결과를 비교한다.
 *    정수화로 답이 바뀌는 지점을 찾기 위한 것이다.
 *    (68000 에는 float 가 없으므로 이 경로는 검증용일 뿐이다)
 * ============================================================ */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "dusk.h"
#include "gelu_tab.h"

/* ── 모델 ─────────────────────────────────────────────────
 *  RCHL 형식: rachel_cpp/export_weights.py 가 만든다.
 *    magic "RCHL" + version + dim,layers,heads,ctx,vocab
 *    어휘 (길이접두 UTF-8)
 *    텐서들 (이름 + 원소수 + scale + int8 데이터)
 */
#define RA_MAXTENSOR 128
#define RA_MAXNAME   64

typedef struct {
    char     name[RA_MAXNAME];
    int8_t  *q;          /* int8 가중치 */
    int      n;          /* 원소 수 */
    int32_t  scale_q;    /* scale 을 q16 정수로 (float 안 씀) */
} RaTensor;

typedef struct {
    int dim, layers, heads, ctx, vocab;
    RaTensor t[RA_MAXTENSOR];
    int      nt;
    char   **itos;       /* 글자표 */
    int8_t  *blob;       /* 가중치 통짜 (한 번에 free) */
    size_t   blob_sz;
} RaModel;

static RaModel g_m;

/* 리틀엔디언 읽기 — 68000 은 빅엔디언이라 실물에서는 바꿔야 한다.
   지금은 PC 에서 도는 시뮬이므로 그대로 읽는다. */
static uint32_t rd_u32(FILE *f) {
    unsigned char b[4];
    if (fread(b, 1, 4, f) != 4) return 0;
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8)
         | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static uint16_t rd_u16(FILE *f) {
    unsigned char b[2];
    if (fread(b, 1, 2, f) != 2) return 0;
    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}
/* scale 은 파일에 fp32 로 들어 있다. 읽기만 하고 곧바로
   q16 정수로 바꾼다 — 이후 계산에는 float 가 안 나온다. */
/* ★ q30 으로 읽는다 — q16 은 작은 스케일을 못 담는다 (실측):
     ln1.bias 의 scale = 0.00013072 → q16 이면 8.57 이 8 로 잘려 6.6% 오차.
     그게 LayerNorm 출력의 작은 원소를 부호까지 뒤집었다 (+0.00093 → -0.00464).
   q30 이면 같은 값이 140,353 이 되어 오차가 0.0004% 로 줄어든다. */
/* ★ 가중치 스케일의 고정소수점 자릿수.
     q16(65536배)으로는 작은 스케일을 못 담는다 — 실측:
       ln1.bias scale = 0.00013072 → q16 이면 8.57 이 8 로 잘려 6.6% 오차.
       그게 LayerNorm 의 작은 원소를 부호까지 뒤집었다
       (h[5] 정답 +0.00093 → dusk -0.00464).
     q30 이면 같은 값이 140,359 가 되어 오차가 0.0004% 로 줄어든다.

   ★ 이 값을 바꾸면 아래 SC_ONE 을 쓰는 곳이 전부 따라 바뀐다.
     전에는 65536 이 코드에 흩어져 있어서 한 곳만 고치면 깨졌다. */
#define SC_BITS 30
#define SC_ONE  (1LL << SC_BITS)

static int32_t rd_scale_q16(FILE *f) {
    unsigned char b[4];
    if (fread(b, 1, 4, f) != 4) return 0;
    uint32_t bits = (uint32_t)b[0] | ((uint32_t)b[1] << 8)
                  | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    /* IEEE754 를 정수 연산만으로 q16 으로 (float 변수 안 씀) */
    int      sign = (bits >> 31) & 1;
    int      exp  = (int)((bits >> 23) & 0xff) - 127;
    uint32_t man  = (bits & 0x7fffff) | 0x800000;   /* 1.xxx, 24비트 */
    /* 값 = man / 2^23 * 2^exp.  q30 으로 = man * 2^(exp - 23 + 30) */
    int sh = exp - 23 + SC_BITS;
    int64_t v = (int64_t)man;
    if (sh > 0) v <<= sh; else if (sh < 0) v >>= (-sh);
    if (sign) v = -v;
    if (v >  0x7fffffffLL) v =  0x7fffffffLL;
    if (v < -0x7fffffffLL) v = -0x7fffffffLL;
    return (int32_t)v;
}

static RaTensor *ra_find(const char *name)
{
    for (int i = 0; i < g_m.nt; i++)
        if (!strcmp(g_m.t[i].name, name)) return &g_m.t[i];
    return NULL;
}

int rachel_load(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "가중치를 못 엽니다: %s\n", path); return 0; }

    char magic[5] = {0};
    if (fread(magic, 1, 4, f) != 4 || strcmp(magic, "RCHL")) {
        fprintf(stderr, "RCHL 형식이 아닙니다\n"); fclose(f); return 0;
    }
    rd_u32(f);                       /* version */
    g_m.dim    = (int)rd_u32(f);
    g_m.layers = (int)rd_u32(f);
    g_m.heads  = (int)rd_u32(f);
    g_m.ctx    = (int)rd_u32(f);
    g_m.vocab  = (int)rd_u32(f);

    /* 어휘 — 전체 바이트수(u32) 다음에 [길이 u8 + UTF-8] 이 이어진다.
       export_weights.py 를 그대로 따른다. */
    uint32_t vb = rd_u32(f);
    unsigned char *vbuf = (unsigned char *)malloc(vb ? vb : 1);
    if (fread(vbuf, 1, vb, f) != vb) { free(vbuf); fclose(f); return 0; }
    g_m.itos = (char **)calloc((size_t)g_m.vocab, sizeof(char *));
    {
        uint32_t o = 0;
        for (int i = 0; i < g_m.vocab && o < vb; i++) {
            int len = vbuf[o++];
            char *sc = (char *)malloc((size_t)len + 1);
            memcpy(sc, vbuf + o, (size_t)len);
            sc[len] = 0;
            o += (uint32_t)len;
            g_m.itos[i] = sc;
        }
    }
    free(vbuf);

    /* 텐서 — 개수(u32) 다음에
       [이름길이 u16 + 이름 + 원소수 u32 + scale fp32 + int8 데이터] */
    uint32_t nt = rd_u32(f);
    if (nt > RA_MAXTENSOR) nt = RA_MAXTENSOR;
    g_m.nt = 0;
    size_t total = 0;
    long body = ftell(f);

    /* 1차: 총 크기 */
    for (uint32_t i = 0; i < nt; i++) {
        uint16_t nl = rd_u16(f);
        if (nl == 0 || nl >= RA_MAXNAME) break;
        fseek(f, nl, SEEK_CUR);
        uint32_t n = rd_u32(f);
        fseek(f, 4 + (long)n, SEEK_CUR);
        total += n;
    }
    g_m.blob = (int8_t *)malloc(total ? total : 1);
    g_m.blob_sz = total;

    /* 2차: 실제 읽기 */
    fseek(f, body, SEEK_SET);
    size_t off = 0;
    for (uint32_t i = 0; i < nt; i++) {
        uint16_t nl = rd_u16(f);
        if (nl == 0 || nl >= RA_MAXNAME) break;
        RaTensor *t = &g_m.t[g_m.nt];
        if (fread(t->name, 1, nl, f) != nl) break;
        t->name[nl] = 0;
        t->n = (int)rd_u32(f);
        t->scale_q = rd_scale_q16(f);
        t->q = g_m.blob + off;
        if (fread(t->q, 1, (size_t)t->n, f) != (size_t)t->n) break;
        off += (size_t)t->n;
        g_m.nt++;
    }
    fclose(f);

    printf("  RACHEL: d%d L%d H%d ctx%d 어휘%d · 텐서 %d개 · %.1fMB\n",
           g_m.dim, g_m.layers, g_m.heads, g_m.ctx, g_m.vocab,
           g_m.nt, (double)g_m.blob_sz / 1048576.0);
    return 1;
}

/* ── 추론 상태 ────────────────────────────────────────────
 *  KV 캐시. 68000 에서는 노드마다 자기 몫만 들고 있으면 된다.
 *  여기서는 시뮬이므로 통째로 잡는다.
 */
typedef struct {
    q15_t  *k, *v;       /* [ctx][dim] */
    int    *kmant, *kshift, *vmant, *vshift;  /* ★ 그 시점의 스케일.
                            위치마다 다르다 — 현재 토큰 스케일을 과거에
                            적용하면 긴 답의 뒤쪽이 틀어진다 (실측:
                            갈리는 위치가 37~93 에 몰렸다). */
    int    *kexp, *vexp; /* 위치마다 지수가 다르다 (블록 부동소수점).
                            ★ 이걸 안 들고 있으면 서로 다른 배율의
                              k 를 그대로 비교하게 된다 — 어텐션이
                              문맥을 못 읽던 원인. */
    int     len;
} RaKV;

/* 어레이 시뮬. 행렬 한 줄씩 노드에 나눠 맡기고, 가장 느린 노드가
   시간을 결정한다 (렌더러에서 배운 것과 같다 — 전체÷노드수가 아니다). */
static Dusk   g_dev;
static int    g_dev_ready = 0;

static RaKV   *g_kv;
static q15_t  *g_x, *g_h, *g_q, *g_k, *g_v, *g_o, *g_mlp;
static q31_t  *g_acc, *g_acc4;
static q15_t  *g_att;

int rachel_init(int nodes, const char *chip, double mhz, const char *topo)
{
    dusk_init(&g_dev, nodes, chip, mhz, topo);
    g_dev_ready = 1;

    int d = g_m.dim, L = g_m.layers, C = g_m.ctx;
    g_kv = (RaKV *)calloc((size_t)L, sizeof(RaKV));
    for (int l = 0; l < L; l++) {
        g_kv[l].k = (q15_t *)calloc((size_t)C * d, sizeof(q15_t));
        g_kv[l].v = (q15_t *)calloc((size_t)C * d, sizeof(q15_t));
        g_kv[l].kexp = (int *)calloc((size_t)C, sizeof(int));
        g_kv[l].vexp = (int *)calloc((size_t)C, sizeof(int));
        g_kv[l].kmant  = (int *)calloc((size_t)C, sizeof(int));
        g_kv[l].kshift = (int *)calloc((size_t)C, sizeof(int));
        g_kv[l].vmant  = (int *)calloc((size_t)C, sizeof(int));
        g_kv[l].vshift = (int *)calloc((size_t)C, sizeof(int));
    }
    g_x   = (q15_t *)calloc((size_t)d, sizeof(q15_t));
    g_h   = (q15_t *)calloc((size_t)d, sizeof(q15_t));
    g_q   = (q15_t *)calloc((size_t)d, sizeof(q15_t));
    g_k   = (q15_t *)calloc((size_t)d, sizeof(q15_t));
    g_v   = (q15_t *)calloc((size_t)d, sizeof(q15_t));
    g_o   = (q15_t *)calloc((size_t)d, sizeof(q15_t));
    g_mlp = (q15_t *)calloc((size_t)d * 4, sizeof(q15_t));
    g_acc = (q31_t *)calloc((size_t)d * 3, sizeof(q31_t));
    g_acc4= (q31_t *)calloc((size_t)d * 4, sizeof(q31_t));
    g_att = (q15_t *)calloc((size_t)C, sizeof(q15_t));
    return 1;
}

void rachel_reset(void)
{
    for (int l = 0; l < g_m.layers; l++) g_kv[l].len = 0;
}


/* q15 두 벡터의 내적 (어텐션 점수용). 결과는 q30 급. */
/* q15 두 벡터의 내적 (어텐션 점수용).

   ★★ 폭을 줄여서 받는다 — 실측으로 잡은 결정적 버그:
     q,k 를 Q15(±32767) 그대로 곱하면 hd=40 개 합이
     40 × 32767² = 429억 으로 int32(21억)를 넘는다.
     실제로 -933,560,989 (부호 뒤집힘) 이 나와 점수가
     +1.32 여야 할 자리에 -0.37 이 나왔다. 어텐션이 무너진 진짜 원인.

     안전 한계는 sqrt(2^31/hd) = ±7327 이다. ±5000 으로 여유를 둔다
     (40 × 5000² = 10억, 안전).

     68000 의 실제 제약이다 — MULS 는 16×16→32, 누산은 ADD.L(32비트).
     정밀도가 15비트에서 12비트로 줄지만, C++ 정답으로 검증한 결과
     200/200 이 나온다 (REFERENCE_int.cpp 참조). */

/* 한 토큰 전방 계산. logits 는 q31 로 돌려준다 (argmax 만 하면 되므로
   정확한 스케일이 필요 없다). */
/* ============================================================
 *  한 토큰 전방 계산 — 검증된 정수 설계 (REFERENCE_int.cpp 200/200)
 *
 *  ★ 스케일을 층 사이로 들고 다니지 않는다.
 *    각 연산마다 입력 최대값으로 스케일을 만들고, 정수 곱셈을 한 뒤,
 *    결과를 바로 되돌린다. 지수 추적 방식은 오차가 쌓여 20% 였다.
 *
 *  활성값은 x_v[] (int32) + x_s (Scale) 로 표현한다.
 *    실제 값 = x_v[i] * x_s.mant / 2^x_s.shift
 * ============================================================ */

/* 스케일 = mant / 2^shift. 68000 에 float 이 없어 이렇게 쓴다. */
typedef struct { int32_t mant; int shift; } Scale;

/* num/den 을 Scale 로. 가수를 15비트에 꽉 채워 정밀도를 지킨다. */
static Scale mk_scale(int64_t num, int64_t den)
{
    Scale s; s.mant = 0; s.shift = 0;
    if (den == 0 || num == 0) return s;
    int neg = 0;
    if (num < 0) { num = -num; neg = 1; }
    if (den < 0) { den = -den; neg = !neg; }

    /* ★ int64 를 넘기지 않고 num/den 을 [2^28, 2^30) 로 맞춘다.
         전에는 den*268435456 을 비교했는데, den 이 크면 그 곱이
         먼저 int64 를 넘어 조건이 깨졌다 (q30 전환 때 터진 자리).
         한쪽만 옮기며 비교해 넘침을 피한다. */
    const int64_t LO = 268435456LL;     /* 2^28 */
    const int64_t HI = 1073741824LL;    /* 2^30 */
    int sh = 0;
    /* num/den 이 LO 보다 작으면 num 을 키우거나 den 을 줄인다 */
    while (sh < 62) {
        if (num >= den && num / den >= LO) break;      /* 충분히 크다 */
        if (num <= (int64_t)4611686018427387903LL) {   /* num<<1 안전 */
            num <<= 1; sh++;
        } else if ((den & 1) == 0) {
            den >>= 1; sh++;
        } else break;                                   /* 더는 못 키운다 */
    }
    /* 너무 크면 den 을 키우거나 num 을 줄인다 */
    while (sh > -62) {
        if (num / den <= HI) break;
        if (den <= (int64_t)4611686018427387903LL) { den <<= 1; sh--; }
        else { num >>= 1; sh--; }
    }

    int64_t m = num / den;
    if (m > 1073741823LL) m = 1073741823LL;
    s.mant  = (int32_t)(neg ? -m : m);
    s.shift = sh;
    return s;
}

/* 두 스케일의 곱.
   ★ 가수를 15비트 근처로 되돌려 정규화한다 — 안 하면 곱할 때마다
     가수가 커지고 shift 가 누적되어(실측 61까지) 배율이 40만 배
     틀어진다. 잔차에서 한쪽 항이 통째로 묻혔던 원인이다. */
static Scale mul_scale(Scale a, Scale b)
{
    Scale s;
    int64_t m = (int64_t)a.mant * b.mant;
    int sh = a.shift + b.shift;
    if (m == 0) { s.mant = 0; s.shift = 0; return s; }
    int neg = m < 0; if (neg) m = -m;
    /* 가수를 [2^28, 2^30) 로 정규화 (30비트 정밀도) */
    while (m >= 1073741824LL) { m >>= 1; sh--; }
    while (m <   268435456LL) { m <<= 1; sh++; }
    s.mant = (int32_t)(neg ? -m : m);
    s.shift = sh;
    return s;
}

/* ── 정수 전용 스케일 도구 (68000 에는 FPU 가 없다) ──────────
   pow(2,shift) 는 시프트 그 자체다. 나눗셈은 mk_scale 안에서 한 번뿐. */

/* a 에 스케일 s 를 곱해 int64 로. shift 는 그냥 시프트 (반올림 포함). */
static int64_t sc_mul(int64_t a, Scale s)
{
    int64_t r = a * s.mant;
    if (s.shift > 0) {
        int sh = s.shift > 62 ? 62 : s.shift;
        r += (r >= 0 ? (1LL << sh) / 2 : -((1LL << sh) / 2));
        r >>= sh;
    } else if (s.shift < 0) r <<= (-s.shift > 30 ? 30 : -s.shift);
    return r;
}

/* sc_mul 인데 a 가 커도 int64 를 안 넘긴다.
   a 를 반으로 줄이면서 shift 를 하나씩 빼면 실제값은 같다. */
static int64_t sc_mul_safe(int64_t a, Scale s)
{
    int64_t aa = a < 0 ? -a : a;
    while (aa > 4294967295LL) { a >>= 1; aa >>= 1; s.shift--; }
    return sc_mul(a, s);
}

/* 두 스케일의 비 a/b (나눗셈 1회) */
static Scale sc_div(Scale a, Scale b)
{
    if (b.mant == 0) return mk_scale(0, 1);
    Scale r = mk_scale(a.mant, b.mant);
    r.shift += a.shift - b.shift;
    return r;
}

/* 활성값 벡터: 값(int32) + 공통 스케일 */
static q31_t *xv, *hv, *qv, *kv_, *vv, *ov, *mv;
static Scale  xs_, hs_, qs_, ks_, vs_;
static q31_t *accv;
static int    ra_bufs = 0;

static void ra_alloc(int d)
{
    if (ra_bufs) return;
    xv = (q31_t*)calloc((size_t)d*4, sizeof(q31_t));
    hv = (q31_t*)calloc((size_t)d*4, sizeof(q31_t));
    qv = (q31_t*)calloc((size_t)d, sizeof(q31_t));
    kv_= (q31_t*)calloc((size_t)d, sizeof(q31_t));
    vv = (q31_t*)calloc((size_t)d, sizeof(q31_t));
    ov = (q31_t*)calloc((size_t)d, sizeof(q31_t));
    mv = (q31_t*)calloc((size_t)d*4, sizeof(q31_t));
    accv=(q31_t*)calloc((size_t)d*4, sizeof(q31_t));
    ra_bufs = 1;
}

/* 벡터를 int16 폭(lim)으로 눌러 양자화한다. 반환값은 '눌린 배율'. */
static Scale quant_to(const q31_t *v, int n, int lim, int16_t *out)
{
    int32_t mx = 0;
    for (int i = 0; i < n; i++) { int32_t a = v[i]<0?-v[i]:v[i]; if (a>mx) mx=a; }
    if (mx == 0) { for (int i=0;i<n;i++) out[i]=0; return mk_scale(1,1); }
    for (int i = 0; i < n; i++) {
        int64_t q = ((int64_t)v[i] * lim + (v[i]>=0?mx/2:-(mx/2))) / mx;
        if (q >  32767) q =  32767;
        if (q < -32768) q = -32768;
        out[i] = (int16_t)q;
    }
    Scale rs = mk_scale(mx, lim);
    return rs;          /* 되돌릴 배율 */
}

/* 행렬곱: y = W·x.  x 는 (xv, xsc), 결과는 (y, ysc) */
static void ra_mv(const RaTensor *W, const q31_t *x, Scale xsc,
                  q31_t *y, Scale *ysc, int rows, int cols)
{
    static int16_t *xq = NULL; static int xq_n = 0;
    if (xq_n < cols) { free(xq); xq = (int16_t*)malloc((size_t)cols*2); xq_n = cols; }

    Scale qs = quant_to(x, cols, 32767, xq);
    if (!W || !W->q) { for (int i=0;i<rows;i++) y[i]=0; *ysc = mk_scale(1,1); return; }

    for (int i = 0; i < rows; i++) {
        const int8_t *w = W->q + (size_t)i*cols;
        /* ★ int64 누산 — 1280개(mlp.2)면 int32 를 넘친다 (실측 53억).
             68000 에서는 ADD.L + ADDX.L 두 명령이다. */
        int64_t acc = 0;
        int j = 0;
        for (; j+3 < cols; j += 4) {
            acc += (int64_t)w[j]  *xq[j];   acc += (int64_t)w[j+1]*xq[j+1];
            acc += (int64_t)w[j+2]*xq[j+2]; acc += (int64_t)w[j+3]*xq[j+3];
        }
        for (; j < cols; j++) acc += (int64_t)w[j]*xq[j];
        if (acc >  2147483647LL || acc < -2147483648LL) {
        }
        if (acc >  2147483647LL) acc =  2147483647LL;
        if (acc < -2147483648LL) acc = -2147483648LL;
        y[i] = (q31_t)acc;
    }
    /* 결과 스케일 = 입력스케일 × 양자화배율 × 가중치스케일 */
    *ysc = mul_scale(mul_scale(xsc, qs), mk_scale(W->scale_q, SC_ONE));

    if (g_dev_ready) {
        DuskCmd c; memset(&c, 0, sizeof c);
        c.op = DUSK_MATVEC; c.rows = rows; c.cols = cols;
        dusk_submit(&g_dev, &c);
    }
}

/* bias 더하기.
   y 의 실제값 = y[i] * ysc.  bias 실제값 = B->q[i]*B->scale_q/65536.
   따라서 y[i] 에 더할 양 = bias실제값 / ysc 다. */
static void ra_bias(q31_t *y, Scale ysc, const RaTensor *B, int n)
{
    if (!B || !B->q || B->n < n || ysc.mant == 0) return;
    /* ★ 정수 전용 (마지막으로 남아 있던 실수 연산 4줄이었다).
         y 실제값 = y[i] x ysc.  bias 실제값 = B->q[i] x scale_q / SC_ONE.
         y 단위로 더할 양 = bias실제값 / ysc.
         배율 F = (1/SC_ONE) / ysc 를 Scale 로 한 번 만들고(나눗셈 1회),
         원소마다 곱셈+시프트 한 번씩이다 (68000: MULS + ASR). */
    Scale F = sc_div(mk_scale(1, SC_ONE), ysc);
    for (int i = 0; i < n; i++) {
        int64_t num = (int64_t)B->q[i] * B->scale_q;      /* 최대 2^37 */
        int64_t add = sc_mul_safe(num, F);
        int64_t v = (int64_t)y[i] + add;
        if (v >  2147483647LL) v =  2147483647LL;
        if (v < -2147483648LL) v = -2147483648LL;
        y[i] = (q31_t)v;
    }
}

/* LayerNorm — 정수. 출력 스케일은 1/4096 로 고정한다 (±8σ 담김) */
#define LN_UNIT 4096
static void ra_ln(q31_t *x, Scale *xsc, int n,
                  const RaTensor *g, const RaTensor *b)
{
    /* 평균·분산은 스케일과 무관하다 (정규화가 지운다) */
    int64_t sum = 0;
    for (int i = 0; i < n; i++) sum += x[i];
    int64_t mean = sum / n;
    /* ★ 분산 누산이 int64 를 넘친다 — 실측으로 잡은 결정적 버그:
         x 가 13억까지 커지면 d*d 가 1.7e18, 320개 합이 5.5e20 으로
         int64(9.2e18)를 훌쩍 넘는다. 그러면 sd 가 42억(2^32 근처)이라는
         엉뚱한 값이 되고, 정규화 결과가 150배 작아진다.
         (증상: LayerNorm 출력이 ±1 이어야 하는데 ±0.0066)

       해결: 편차를 미리 k칸 줄여서 제곱한다.
         d>>k 로 제곱하면 vs 는 2^(2k) 배 작고, sd 는 2^k 배 작다.
         나중에 nv 계산에서 그만큼 되돌린다.
       68000 관점: 시프트 한 번 추가일 뿐이다. */
    int vsh = 0;
    {
        int32_t dmx = 0;
        for (int i = 0; i < n; i++) {
            int64_t dv = x[i] - mean;
            int32_t a = (int32_t)(dv < 0 ? -dv : dv);
            if (a > dmx) dmx = a;
        }
        /* d^2 * n 이 int64 에 들어가려면 d <= sqrt(2^62/n) 이어야 한다.
           n=320 이면 약 1.2e8 (27비트). 여유를 두고 24비트로 맞춘다. */
        while (dmx > 16777215 && vsh < 30) { dmx >>= 1; vsh++; }
    }
    int64_t vs = 0;
    for (int i = 0; i < n; i++) {
        int64_t d = (x[i] - mean) >> vsh;
        vs += d*d;
    }
    vs /= n;
    if (vs == 0) vs = 1;
    /* 정수 제곱근 */
    uint64_t r = (uint64_t)vs, y0 = r, y1 = (y0+1)>>1;
    while (y1 < y0) { y0 = y1; y1 = (y0 + r/y0) >> 1; }
    int64_t sd = (int64_t)y0; if (sd < 1) sd = 1;

    for (int i = 0; i < n; i++) {
        int64_t d = x[i] - mean;
        /* ★ 나눗셈은 0 쪽으로 버린다 — 작은 편차가 통째로 사라진다.
             실측: h[5] 정답 +0.00093 인데 dusk 는 -0.00464 로 부호까지
             뒤집혔다. 반올림을 넣는다 (68000 에서는 ADD.L 한 번). */
        /* sd 는 2^vsh 배 작으므로 d 도 그만큼 줄여 맞춘다 */
        int64_t dd = vsh ? (d >> vsh) : d;
        int64_t num = dd * LN_UNIT;
        int64_t nv = (num + (num >= 0 ? sd/2 : -(sd/2))) / sd;
        /* ★ >>(7+16) 이면 gamma 가 128 로 더 나눠진다 — int8 을 실수로
           되돌리는 몫은 scale_q 안에 이미 있다. C++ 검증판도 >>16 이다.
           (실측: 이 한 줄 때문에 LayerNorm 출력이 0.0077, 정답의 1/118) */
        /* ★ gamma = g[i] * scale_q / SC_ONE.  실제로는 1.0 근처다.
             그냥 >>SC_BITS 하면 nv 가 작을 때 전부 0 이 된다.
             반올림을 넣고, int8 몫(g[i]) 을 먼저 곱해 자리를 지킨다. */
        if (g && g->q) {
            int64_t t = nv * g->q[i] * g->scale_q;
            t += (t >= 0 ? (1LL<<(SC_BITS-1)) : -(1LL<<(SC_BITS-1)));
            nv = t >> SC_BITS;
        }
        if (b && b->q) {
            int64_t t = (int64_t)b->q[i] * b->scale_q * LN_UNIT;
            t += (t >= 0 ? (1LL<<(SC_BITS-1)) : -(1LL<<(SC_BITS-1)));
            nv += t >> SC_BITS;
        }
        if (nv >  2147483647LL) nv =  2147483647LL;
        if (nv < -2147483648LL) nv = -2147483648LL;
        x[i] = (q31_t)nv;
    }
    *xsc = mk_scale(1, LN_UNIT);       /* 값 1 = 1/LN_UNIT */
}

static void ra_step(int tok, int pos, q31_t *logits)
{
    int d = g_m.dim, H = g_m.heads, hd = d / H, C = g_m.ctx;
    ra_alloc(d);

    RaTensor *emb = ra_find("tok.weight");
    RaTensor *pe  = ra_find("pos.weight");

    /* ── 임베딩 + 위치 ── */
    {
        int32_t es = emb ? emb->scale_q : (int32_t)SC_ONE;
        int32_t ps = pe  ? pe->scale_q  : (int32_t)SC_ONE;
        int32_t hs = es > ps ? es : ps;
        for (int i = 0; i < d; i++) {
            int32_t e = emb ? emb->q[(size_t)tok*d+i] : 0;
            int32_t p = pe  ? pe->q[(size_t)pos*d+i]  : 0;
            /* ★ 나눗셈을 먼저 하면 정밀도를 잃는다 (q30 에서는 0 이 된다).
                 128 을 먼저 곱하고 나눈다. e,p 는 int8 이라 넘치지 않는다. */
            xv[i] = (q31_t)((((int64_t)e*es + (int64_t)p*ps) * 128) / hs);
        }
        xs_ = mk_scale(hs, SC_ONE*128);
    }

    for (int l = 0; l < g_m.layers; l++) {
        char p[RA_MAXNAME];
        RaKV *c = &g_kv[l];

        /* ── 어텐션 ── */
        memcpy(hv, xv, (size_t)d*sizeof(q31_t)); hs_ = xs_;
        snprintf(p,sizeof p,"blocks.%d.ln1.weight",l); RaTensor *g1=ra_find(p);
        snprintf(p,sizeof p,"blocks.%d.ln1.bias",l);   RaTensor *b1=ra_find(p);
        ra_ln(hv, &hs_, d, g1, b1);

        snprintf(p,sizeof p,"blocks.%d.attn.in_proj_weight",l); RaTensor *Wq=ra_find(p);
        snprintf(p,sizeof p,"blocks.%d.attn.in_proj_bias",l);   RaTensor *Bq=ra_find(p);
        Scale asc;
        ra_mv(Wq, hv, hs_, accv, &asc, 3*d, d);
        ra_bias(accv, asc, Bq, 3*d);

        memcpy(qv,  accv,        (size_t)d*sizeof(q31_t));
        memcpy(kv_, accv+d,      (size_t)d*sizeof(q31_t));
        memcpy(vv,  accv+2*d,    (size_t)d*sizeof(q31_t));
        qs_ = ks_ = vs_ = asc;

        /* KV 캐시 — int16 으로 눌러 저장하고 되돌릴 배율을 기록한다.
           ★ 반올림을 한다 — 잘라내면 긴 답에서 오차가 누적된다. */
        {
            int32_t kmx=0, vmx=0;
            for (int i=0;i<d;i++){
                int32_t a=kv_[i]<0?-kv_[i]:kv_[i]; if(a>kmx)kmx=a;
                int32_t b2=vv[i]<0?-vv[i]:vv[i];   if(b2>vmx)vmx=b2;
            }
            if (kmx == 0) kmx = 1;
            if (vmx == 0) vmx = 1;
            for (int i=0;i<d;i++){
                /* ★ 반올림을 *2 .. /2 로 하면 큰 값에서 int64 를 넘칠
                     수 있다. 나눗셈 결과에 직접 반올림을 얹는다. */
                int64_t kn = (int64_t)kv_[i]*32000, vn = (int64_t)vv[i]*32000;
                int64_t kq = (kn + (kn>=0 ? kmx/2 : -(kmx/2))) / kmx;
                int64_t vq = (vn + (vn>=0 ? vmx/2 : -(vmx/2))) / vmx;
                if (kq >  32767) kq =  32767;
                if (kq < -32768) kq = -32768;
                if (vq >  32767) vq =  32767;
                if (vq < -32768) vq = -32768;
                c->k[(size_t)pos*d+i]=(q15_t)kq;
                c->v[(size_t)pos*d+i]=(q15_t)vq;
            }
            c->kexp[pos] = kmx;      /* 되돌릴 배율 (지수가 아니라 값) */
            c->vexp[pos] = vmx;
            c->kmant[pos]  = ks_.mant;   c->kshift[pos]  = ks_.shift;
            c->vmant[pos]  = vs_.mant;   c->vshift[pos]  = vs_.shift;
        }
        c->len = pos+1;

        /* ── 헤드별 어텐션 ── */
        {
            static int16_t qq[64];
            static q31_t   ihv[4096];     /* ── 정수판 어텐션 출력 (나란히 계산) */
            static int32_t iatt[512];     /* 정수 점수(Q11, int32 클램프 없음) → 확률(Q15) */
            static int64_t ihacc[64];
            for (int i = 0; i < d; i++) { hv[i] = 0; ihv[i] = 0; }
            /* 1/sqrt(hd)=0.158 을 Q15 로, 그리고 Q11 배율 2048 */
            const Scale S_INVSQ = mk_scale(5178, 32768);
            const Scale S_Q11   = mk_scale(2048, 1);

            for (int hh = 0; hh < H; hh++) {
                /* ★ q,k 를 ±5000 으로 눌러 곱한다 — int32 누산 한계.
                     ±32767 이면 40개 합이 429억으로 넘쳐 부호가 뒤집힌다
                     (실측: 점수 +1.32 가 -0.37 로 나왔다). */
                int32_t qmx = 0;
                for (int j=0;j<hd;j++){int32_t a=qv[hh*hd+j]<0?-qv[hh*hd+j]:qv[hh*hd+j]; if(a>qmx)qmx=a;}
                if (qmx==0) qmx=1;
                /* q 도 ±32000 까지 쓴다 (int64 누산이라 넘치지 않는다) */
                for (int j=0;j<hd;j++) qq[j]=(int16_t)((int64_t)qv[hh*hd+j]*32000/qmx);

                /* ── 정수 점수 배율 (헤드 공통 부분) ──────────────
                   실제점수 = acc x (qmx/32000) x qs_ x (kexp/32000) x kmant/2^kshift / sqrt(hd)
                   ★ softmax 는 exp 라 비선형이다 — 배율을 생략하면 분포가
                     달라진다. 덧셈 상수만 뺄 수 있다. 그래서 진짜 배율을
                     Scale 곱으로 전부 만든다 (곱셈·시프트·나눗셈 1회뿐). */
                Scale Fq = mul_scale(mul_scale(mk_scale(qmx, 32000), qs_), S_INVSQ);
                Fq = mul_scale(Fq, S_Q11);          /* Q11 로 담는 배율까지 */

                for (int t2 = 0; t2 <= pos && t2 < C; t2++) {
                    /* ★ int64 누산이므로 폭을 줄일 이유가 없다.
                         k 를 저장된 ±32000 그대로 쓴다 — 정밀도가
                         12비트에서 15비트로 올라간다.
                         (긴 답의 90번째 글자쯤에서 갈리던 문제) */
                    int64_t acc = 0;
                    for (int j=0;j<hd;j++)
                        acc += (int64_t)qq[j] * c->k[(size_t)t2*d+hh*hd+j];

                    /* 정수 점수: 위치별 k 배율을 곱해 Q11 로 */
                    {
                        Scale Fk = mul_scale(mk_scale(c->kexp[t2], 32000),
                                             mk_scale(c->kmant[t2], 1));
                        Fk.shift += c->kshift[t2];     /* kmant/2^kshift */
                        int64_t sq = sc_mul_safe(acc, mul_scale(Fq, Fk));
                        /* ★ int16 클램프를 없앴다 — ±16 을 넘는 점수가 뭉개져
                             L4 에서 오차 65% 가 났다. int32 로 그대로 둔다. */
                        if (sq >  2147483647LL) sq =  2147483647LL;
                        if (sq < -2147483648LL) sq = -2147483648LL;
                        iatt[t2] = (int32_t)sq;
                    }
                }
                /* ── 정수 softmax + 가중합 ──────────────────────── */
                {
                    int n2 = pos + 1 < C ? pos + 1 : C;
                    dusk_softmax_q32(iatt, n2);        /* Q11 점수(int32) → Q15 확률 */
                    for (int j=0;j<hd;j++) ihacc[j] = 0;
                    for (int t2 = 0; t2 < n2; t2++) {
                        if (!iatt[t2]) continue;
                        /* real_v = v x vexp/32000 x vmant/2^vshift.
                           결과를 현재 vs_ 단위로: / vs_.  확률은 /32767. */
                        Scale G = mul_scale(mk_scale(c->vexp[t2], 32000),
                                            mk_scale(c->vmant[t2], 1));
                        G.shift += c->vshift[t2];
                        G = mul_scale(sc_div(G, vs_), mk_scale(1, 32767));
                        for (int j=0;j<hd;j++) {
                            int64_t pv = (int64_t)iatt[t2] * c->v[(size_t)t2*d+hh*hd+j];
                            ihacc[j] += sc_mul_safe(pv, G);
                        }
                    }
                    for (int j=0;j<hd;j++) {
                        int64_t v = ihacc[j];
                        if (v >  2147483647LL) v =  2147483647LL;
                        if (v < -2147483648LL) v = -2147483648LL;
                        ihv[hh*hd+j] = (q31_t)v;
                    }
                }

                /* ★ 정수판을 쓴다 (실수판 대비 벡터기준 오차 0.02~0.3%) */
                for (int j=0;j<hd;j++) hv[hh*hd+j] = ihv[hh*hd+j];
            }
            hs_ = vs_;
        }

        snprintf(p,sizeof p,"blocks.%d.attn.out_proj.weight",l); RaTensor *Wo=ra_find(p);
        snprintf(p,sizeof p,"blocks.%d.attn.out_proj.bias",l);   RaTensor *Bo=ra_find(p);
        Scale osc;
        ra_mv(Wo, hv, hs_, ov, &osc, d, d);
        ra_bias(ov, osc, Bo, d);

        /* 잔차 — 두 스케일을 맞춰 더한다 */
        {

            /* ── 정수판을 나란히 계산해 대조한다 (아직 쓰진 않는다) ──
               실제값 = 값 x mant / 2^shift.
               공통분모를 2^cs(작은 쪽 shift)로 잡으면 오른쪽 시프트만
               나온다. 가수를 곱하면 2^30 배 커지므로 >>30 으로 되돌린다. */
            static q31_t ixv[4096]; Scale ixs;
            {
                int cs = xs_.shift < osc.shift ? xs_.shift : osc.shift;
                int sx = xs_.shift - cs, so = osc.shift - cs;
                if (sx > 40) sx = 40;
                if (so > 40) so = 40;
                /* 두 항이 시프트 뒤 얼마나 작아지는지 미리 보고,
                   int32 를 꽉 채우도록 덜 밀 칸수(keep)를 정한다. */
                int keep = 0;
                {
                    int64_t pk = 0;
                    for (int i = 0; i < d; i++) {
                        int64_t a = (int64_t)xv[i] * xs_.mant;
                        int64_t b = (int64_t)ov[i] * osc.mant;
                        int t0 = 30 + sx, t1 = 30 + so;
                        if (t0 > 62) t0 = 62;
                        if (t1 > 62) t1 = 62;
                        int64_t t = ((a>>t0) < 0 ? -(a>>t0) : (a>>t0))
                                  + ((b>>t1) < 0 ? -(b>>t1) : (b>>t1));
                        if (t > pk) pk = t;
                    }
                    /* int32 상한(2^31)까지 채운다 — 2^30 에서 멈추면 절반을 버린다 */
                    while (pk > 0 && keep < 30 && (pk << (keep+1)) < 2000000000LL) keep++;
                }
                static int64_t tmp[4096];
                int64_t mx = 0;
                for (int i = 0; i < d; i++) {
                    /* ★ 가수 곱셈 되돌림(>>30)과 공통분모 맞춤(>>sx)을
                         한 번에 한다 — 두 번 나눠 밀면 반올림 기회를
                         두 번 잃어 작은 값이 뭉개진다 (실측: 작은 원소에서
                         오차 71%). 시프트를 합치고 한 번만 반올림한다. */
                    /* ★ 시프트 양을 미리 줄여 자리를 남긴다 — 실측:
                         >>30 >>sx 를 그대로 하면 합이 4984 까지 떨어져
                         (int32 의 0.0002%) 작은 값이 통째로 사라진다.
                         나중에 dn<0 으로 키워봐야 이미 잃은 뒤다.
                         공통으로 KEEP 칸을 덜 밀어 자리를 확보한다. */
                    int ta = 30 + sx - keep, tb = 30 + so - keep;
                    if (ta < 0) ta = 0;
                    if (tb < 0) tb = 0;
                    if (ta > 62) ta = 62;
                    if (tb > 62) tb = 62;
                    int64_t a = (int64_t)xv[i] * xs_.mant;
                    int64_t b = (int64_t)ov[i] * osc.mant;
                    a = (a + (a>=0 ? (1LL<<ta)/2 : -((1LL<<ta)/2))) >> ta;
                    b = (b + (b>=0 ? (1LL<<tb)/2 : -((1LL<<tb)/2))) >> tb;
                    tmp[i] = a + b;
                    int64_t t = tmp[i] < 0 ? -tmp[i] : tmp[i];
                    if (t > mx) mx = t;
                }
                int dn = 0;
                while ((mx >> dn) > 2147483647LL && dn < 40) dn++;
                if (dn == 0)
                    while (mx > 0 && dn > -30 && (mx << (1-dn)) < 1073741824LL) dn--;
                for (int i = 0; i < d; i++) {
                    int64_t v = tmp[i];
                    if (dn > 0)      v = (v + (v>=0 ? (1LL<<dn)/2 : -((1LL<<dn)/2))) >> dn;
                    else if (dn < 0) v <<= (-dn);
                    if (v >  2147483647LL) v =  2147483647LL;
                    if (v < -2147483648LL) v = -2147483648LL;
                    ixv[i] = (q31_t)v;
                }
                ixs.mant  = 536870912;              /* 2^29 */
                /* keep 만큼 덜 밀었으니 값이 2^keep 배 크다 -> 분모도 그만큼 */
                ixs.shift = cs - dn + keep + 29 - 30;
            }

            /* ★ 정수판을 쓴다 */
            for (int i = 0; i < d; i++) xv[i] = ixv[i];
            xs_ = ixs;

        }

        /* ── MLP ── */
        memcpy(hv, xv, (size_t)d*sizeof(q31_t)); hs_ = xs_;
        snprintf(p,sizeof p,"blocks.%d.ln2.weight",l); RaTensor *g2=ra_find(p);
        snprintf(p,sizeof p,"blocks.%d.ln2.bias",l);   RaTensor *b2=ra_find(p);
        ra_ln(hv, &hs_, d, g2, b2);

        snprintf(p,sizeof p,"blocks.%d.mlp.0.weight",l); RaTensor *W1=ra_find(p);
        snprintf(p,sizeof p,"blocks.%d.mlp.0.bias",l);   RaTensor *B1=ra_find(p);
        Scale msc;
        ra_mv(W1, hv, hs_, mv, &msc, 4*d, d);
        ra_bias(mv, msc, B1, 4*d);

        /* ── GELU ── 정수판을 나란히 계산해 대조 (아직 안 씀) ─────
           표는 ±4.0 을 32768 로 담는다 (Q15 의 1/4 규약).
             표입력 = 실제값 x 8192 = mv * msc.mant * 2^13 / 2^msc.shift */
        static q31_t imv[4096]; Scale ims;
        {
            int sh = msc.shift - 13;
            int32_t gmx = 0;
            for (int i = 0; i < 4*d; i++) {
                int64_t t = (int64_t)mv[i] * msc.mant;
                if (sh > 0) {
                    int q = sh > 62 ? 62 : sh;
                    t = (t + (t>=0 ? (1LL<<q)/2 : -((1LL<<q)/2))) >> q;
                } else if (sh < 0) t <<= (-sh > 30 ? 30 : -sh);
                if (t >  32767) t =  32767;
                if (t < -32768) t = -32768;
                int32_t k  = (int32_t)(t >> 7);
                int32_t f  = (int32_t)(t & 127);
                int32_t a  = GELU_TAB[k & 511];
                int32_t b2 = GELU_TAB[(k + 1) & 511];
                int32_t g  = a + (((b2 - a) * f) >> 7);
                imv[i] = g;
                int32_t ag = g < 0 ? -g : g;
                if (ag > gmx) gmx = ag;
            }
            /* 표 출력은 ±32767 이라 int32 를 거의 안 쓴다 — 자리를 채운다 */
            int up = 0;
            while (gmx > 0 && up < 20 && ((int64_t)gmx << (up+1)) < 2000000000LL) up++;
            if (up) for (int i = 0; i < 4*d; i++) imv[i] <<= up;
            /* 단위: 실제값 x 8192 x 2^up  ->  배율 1/(8192 x 2^up) */
            ims.mant  = 536870912;          /* 2^29 */
            ims.shift = 29 + 13 + up;
        }

        /* ★ 정수판(표 + 시프트)을 쓴다. 실수판 대비 오차 0.008%. */
        for (int i = 0; i < 4*d; i++) mv[i] = imv[i];
        msc = ims;

        snprintf(p,sizeof p,"blocks.%d.mlp.2.weight",l); RaTensor *W2=ra_find(p);
        snprintf(p,sizeof p,"blocks.%d.mlp.2.bias",l);   RaTensor *B2=ra_find(p);
        Scale o2sc;
        ra_mv(W2, mv, msc, ov, &o2sc, d, 4*d);
        ra_bias(ov, o2sc, B2, d);

        {
            /* ★ MLP 잔차도 같은 정수 방식 (attn 잔차와 짝을 이뤄야 한다 —
                 한쪽만 바꾸면 스케일 규약이 어긋나 값이 폭발한다.
                 실측: attn 만 바꿨더니 L1 에서 1600배가 됐다). */
            {
                int cs = xs_.shift < o2sc.shift ? xs_.shift : o2sc.shift;
                int sx = xs_.shift - cs, so = o2sc.shift - cs;
                if (sx > 40) sx = 40;
                if (so > 40) so = 40;
                int keep = 0;
                {
                    int64_t pk = 0;
                    for (int i = 0; i < d; i++) {
                        int t0 = 30 + sx, t1 = 30 + so;
                        if (t0 > 62) t0 = 62;
                        if (t1 > 62) t1 = 62;
                        int64_t a = ((int64_t)xv[i] * xs_.mant) >> t0;
                        int64_t b = ((int64_t)ov[i] * o2sc.mant) >> t1;
                        int64_t t = (a<0?-a:a) + (b<0?-b:b);
                        if (t > pk) pk = t;
                    }
                    while (pk > 0 && keep < 30 && (pk << (keep+1)) < 2000000000LL) keep++;
                }
                static int64_t tmp[4096];
                int64_t mx = 0;
                for (int i = 0; i < d; i++) {
                    int ta = 30 + sx - keep, tb = 30 + so - keep;
                    if (ta < 0) ta = 0;
                    if (tb < 0) tb = 0;
                    if (ta > 62) ta = 62;
                    if (tb > 62) tb = 62;
                    int64_t a = (int64_t)xv[i] * xs_.mant;
                    int64_t b = (int64_t)ov[i] * o2sc.mant;
                    a = (a + (a>=0 ? (1LL<<ta)/2 : -((1LL<<ta)/2))) >> ta;
                    b = (b + (b>=0 ? (1LL<<tb)/2 : -((1LL<<tb)/2))) >> tb;
                    tmp[i] = a + b;
                    int64_t t = tmp[i] < 0 ? -tmp[i] : tmp[i];
                    if (t > mx) mx = t;
                }
                int dn = 0;
                while ((mx >> dn) > 2147483647LL && dn < 40) dn++;
                for (int i = 0; i < d; i++) {
                    int64_t v = tmp[i];
                    if (dn > 0) v = (v + (v>=0 ? (1LL<<dn)/2 : -((1LL<<dn)/2))) >> dn;
                    if (v >  2147483647LL) v =  2147483647LL;
                    if (v < -2147483648LL) v = -2147483648LL;
                    xv[i] = (q31_t)v;
                }
                xs_.mant  = 536870912;              /* 2^29 */
                xs_.shift = cs - dn + keep + 29 - 30;
            }
        }
    }

    RaTensor *gl = ra_find("ln.weight");
    RaTensor *bl = ra_find("ln.bias");
    ra_ln(xv, &xs_, d, gl, bl);
    Scale lsc;
    ra_mv(emb, xv, xs_, logits, &lsc, g_m.vocab, d);
}

/* ── 글자 <-> 번호 ─────────────────────────────────────── */

static int ra_lookup(const char *ch)
{
    for (int i = 0; i < g_m.vocab; i++)
        if (g_m.itos[i] && !strcmp(g_m.itos[i], ch)) return i;
    return 1;      /* <unk> */
}

/* UTF-8 한 글자 길이 */
static int u8len(unsigned char c)
{
    if (c < 0x80) return 1;
    if ((c & 0xe0) == 0xc0) return 2;
    if ((c & 0xf0) == 0xe0) return 3;
    return 4;
}

/* 프롬프트를 넣고 답을 생성한다. out 에 UTF-8 로 쓴다. */
int rachel_run(const char *prompt, char *out, int outmax, long long *cycles)
{
    rachel_reset();
    int pos = 0;
    q31_t *logits = (q31_t *)malloc((size_t)g_m.vocab * sizeof(q31_t));
    if (g_dev_ready) dusk_reset_counters(&g_dev);

    /* 프롬프트 밀어넣기 */
    const char *p = prompt;
    while (*p && pos < g_m.ctx - 1) {
        char ch[5] = {0};
        int n = u8len((unsigned char)*p);
        memcpy(ch, p, (size_t)n);
        ra_step(ra_lookup(ch), pos, logits);
        p += n;
        pos++;
    }

    /* 생성 */
    int wr = 0;
    int dbg = getenv("DUSK_DBG") != NULL;
    while (pos < g_m.ctx - 1 && wr < outmax - 5) {
        int best = 0;
        q31_t bv = logits[0];
        for (int i = 1; i < g_m.vocab; i++)
            if (logits[i] > bv) { bv = logits[i]; best = i; }
        if (dbg && wr < 40) {
            /* 상위 3개를 본다 — 정수화가 어디서 깨지는지 찾기 위해 */
            int i1 = best, i2 = -1, i3 = -1;
            q31_t v2 = -2147483647, v3 = -2147483647;
            for (int i = 0; i < g_m.vocab; i++) {
                if (i == i1) continue;
                if (logits[i] > v2) { v3=v2; i3=i2; v2=logits[i]; i2=i; }
                else if (logits[i] > v3) { v3=logits[i]; i3=i; }
            }
            fprintf(stderr, "  pos%d: [%s]%d  [%s]%d  [%s]%d\n", pos,
                g_m.itos[i1]?g_m.itos[i1]:"?", (int)bv,
                (i2>=0&&g_m.itos[i2])?g_m.itos[i2]:"?", (int)v2,
                (i3>=0&&g_m.itos[i3])?g_m.itos[i3]:"?", (int)v3);
        }
        if (getenv("DUSK_RANK") && wr == 0) {
            /* 첫 글자에서 특정 글자의 순위를 본다 (정답 대조용) */
            const char *want = getenv("DUSK_RANK");
            int wi = -1;
            for (int i = 0; i < g_m.vocab; i++)
                if (g_m.itos[i] && !strcmp(g_m.itos[i], want)) { wi = i; break; }
            if (wi >= 0) {
                int rank = 1;
                for (int i = 0; i < g_m.vocab; i++)
                    if (logits[i] > logits[wi]) rank++;
                fprintf(stderr, "  '%s' 순위 %d위 (logit %d) · 1위 '%s' %d\n",
                        want, rank, (int)logits[wi],
                        g_m.itos[best]?g_m.itos[best]:"?", (int)bv);
            }
        }
        if (best == 2) break;                    /* <eos> */
        const char *s = g_m.itos[best];
        if (!s) break;
        int sl = (int)strlen(s);
        if (wr + sl >= outmax - 1) break;
        memcpy(out + wr, s, (size_t)sl);
        wr += sl;
        ra_step(best, pos, logits);
        pos++;
    }
    out[wr] = 0;
    free(logits);
    if (cycles) *cycles = g_dev_ready ? dusk_peak_cycles(&g_dev) : 0;
    return wr;
}

int rachel_vocab(void) { return g_m.vocab; }
int rachel_dim(void)   { return g_m.dim; }

/* 마지막 실행이 실제 68000 에서 걸릴 시간 (초).
   dusk_seconds 가 노드 계산 + 통신 + CPU 발행을 합쳐서 준다. */
double rachel_seconds(void)
{
    return g_dev_ready ? dusk_seconds(&g_dev) : 0.0;
}
