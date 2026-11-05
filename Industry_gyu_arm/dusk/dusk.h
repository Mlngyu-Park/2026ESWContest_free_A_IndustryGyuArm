/* ============================================================
 *  DUSK — 68000 뭉치를 GPU 처럼 쓰는 장치
 *
 *  구조:  68000 CPU 하나  +  68000 노드 N개 (DUSK)
 *
 *    CPU 는 계산하지 않는다. 명령(DuskCmd)만 큐에 넣는다.
 *    노드는 공유 메모리를 직접 읽는다. CPU 가 데이터를 나르지 않는다.
 *    ─ 이유: CPU 하나가 32노드에 데이터를 날라주면 CPU 가 병목이 된다.
 *            아미가 블리터와 같은 방식으로 피한다.
 *
 *  쓰는 곳 세 가지 — 전부 같은 명령 집합을 쓴다:
 *    ① RACHEL 언어모델   MATVEC 위주
 *    ② 3D 그래픽         XFORM + RASTER
 *    ③ 역기구학          MATVEC + 삼각함수 (나중)
 *
 *  칩 교체: 이 파일과 dusk.c/dusk_node.c 는 칩과 무관하다.
 *           mac_68k.s 를 mac_6502.s 로 바꾸고 아래 사이클표만 고른다.
 * ============================================================ */
#ifndef DUSK_H
#define DUSK_H

#include <stdint.h>
#include <stddef.h>

/* ── 칩별 사이클표 ─────────────────────────────────────────
 *  칩을 바꿀 때 바꾸는 것은 이 표와 MAC 커널뿐이다.
 *  값 근거: 68000 은 mac_68k.s 실측 100T(언롤4).
 *           나머지는 각 사 매뉴얼 사이클로 계산한 추정치다. */
typedef struct {
    const char *name;
    int   t_mac;        /* MAC 하나 (곱셈+누산+로드+분기) */
    int   t_ring;       /* 링으로 1바이트 전달 */
    int   t_mul;        /* 16x16 곱셈 단독 */
    int   t_div;        /* 32/16 나눗셈 단독 */
    int   t_ld;         /* 메모리 로드 1워드 */
    long  addr_space;   /* 주소공간 바이트 (뱅크 필요 판단) */
    int   has_mul;      /* 하드웨어 곱셈기 있음 */
} DuskChip;

/* 사이클표. 68000 만 실측, 나머지는 매뉴얼 기반 계산값이다. */
static const DuskChip DUSK_CHIPS[] = {
  /* name        mac  ring  mul  div  ld  addr_space   has_mul */
  { "68000",     100,   40,  70, 158,  8,  16777216L,  1 },  /* 실측 */
  { "68000-12",  100,   40,  70, 158,  8,  16777216L,  1 },  /* 같은 T, 클럭만 다름 */
  { "68020",      20,   16,  28,  56,  3,  4294967296L,1 },  /* 캐시·32비트 */
  { "68030",      13,   12,  28,  56,  2,  4294967296L,1 },
  { "z80",      1140,   90,   0,   0,  7,     65536L,  0 },  /* 곱셈 없음: 시프트+덧셈 */
  { "6502",      880,   70,   0,   0,  4,     65536L,  0 },  /* 8비트, 곱셈 없음 */
  { "486dx2",     12,    8,  13,  40,  1,  4294967296L,1 },
};
#define DUSK_NCHIPS ((int)(sizeof(DUSK_CHIPS)/sizeof(DUSK_CHIPS[0])))

/* ── 명령 종류 ─────────────────────────────────────────────
 *  CPU 가 발행하고 노드가 실행한다. 세 용도가 이 집합을 공유한다. */
typedef enum {
    DUSK_NOP = 0,
    DUSK_MATVEC,     /* y = W·x  (LLM·역기구학의 대부분) */
    DUSK_XFORM,      /* 정점을 행렬로 변환 (그래픽 기하) */
    DUSK_RASTER,     /* 삼각형 채우기 (그래픽) */
    DUSK_VADD,       /* 벡터 덧셈 (잔차 연결) */
    DUSK_ACT,        /* 활성함수 GELU/LayerNorm/Softmax */
    DUSK_SYNC,       /* 모든 노드 대기 (배리어) */
} DuskOp;

/* 활성함수 종류 (DUSK_ACT 의 arg) */
typedef enum { ACT_GELU=0, ACT_LAYERNORM=1, ACT_SOFTMAX=2 } DuskAct;

/* ── 명령 하나 ────────────────────────────────────────────
 *  CPU 가 쓰는 것은 이 구조체 하나뿐이다 (16바이트).
 *  실제 데이터는 공유 메모리에 있고 여기엔 주소만 담긴다.
 *  ─ 그래서 CPU 부하가 노드 수와 무관하다. */
typedef struct {
    uint8_t  op;        /* DuskOp */
    uint8_t  arg;       /* 활성함수 종류 등 */
    uint16_t rows;      /* 결과 개수 (이걸 노드들이 나눠 맡는다) */
    uint16_t cols;      /* 입력 개수 */
    uint16_t flags;
    uint32_t src_w;     /* 가중치 주소 (공유 메모리) */
    uint32_t src_x;     /* 입력 벡터 주소 */
    uint32_t dst_y;     /* 출력 벡터 주소 */
    float    scale;     /* int8 -> 실수 복원 배율 */
} DuskCmd;

/* ── 노드 하나의 상태 ───────────────────────────────────── */
#define DUSK_MAXNODE 256

typedef struct {
    int       id;           /* 0..nodes-1 */
    long long cycles;       /* 이 노드가 쓴 사이클 (시간 = 최대값) */
    long long macs;         /* 이 노드가 한 MAC 수 */
    long long bytes_in;     /* 링으로 받은 바이트 */
    long long bytes_out;    /* 링으로 보낸 바이트 */
    long long idle;         /* 놀았던 사이클 (행이 노드보다 적을 때) */
} DuskNode;

/* ── DUSK 장치 전체 ─────────────────────────────────────── */
typedef struct {
    int             nodes;          /* 노드 개수 */
    double          clk_hz;         /* 클럭 */
    const DuskChip *chip;           /* 사이클표 */
    DuskNode        node[DUSK_MAXNODE];
    long long       cpu_cycles;     /* CPU 68000 이 쓴 사이클 ★ 병목 확인용 */
    long long       common_cycles;  /* 아무도 못 나눈 공통 작업 ★ 94% 함정 */
    int             topo;           /* 0=ring 1=group 2=tree */
    long long       ncmd;           /* 발행한 명령 수 */
} Dusk;

/* ── CPU 쪽 API (dusk.c) ────────────────────────────────── */
void   dusk_init(Dusk *D, int nodes, const char *chip, double mhz, const char *topo);
void   dusk_submit(Dusk *D, const DuskCmd *c);   /* CPU 가 명령 발행 */
void   dusk_sync(Dusk *D);                       /* 모든 노드 완료 대기 */
double dusk_seconds(const Dusk *D);              /* 실제 걸리는 시간 */
void   dusk_report(const Dusk *D, const char *title);
void   dusk_reset_counters(Dusk *D);             /* 한 문장마다 0 부터 */
long long dusk_peak_cycles(const Dusk *D);       /* 가장 느린 노드 */
const DuskChip *dusk_find_chip(const char *name);

/* ── 노드 쪽 API (dusk_node.c) — 정수 전용 ─────────────── */
/*  주의: 여기서부터 float 를 쓰면 안 된다. 68000 에 FPU 가 없다. */

/* 활성값 고정소수점. Q15: -1.0 ~ +0.99997 를 int16 으로 */
typedef int16_t q15_t;
typedef int32_t q31_t;

/* ── 블록 부동소수점 (활성값 표현 방식) ────────────────────
 *  ★ 왜 이게 필요한가 — 실측으로 알아낸 것:
 *
 *  RACHEL 의 잔차 연결은 층마다 값이 자란다. C++ 원본을 계측해 보니
 *    L0 rms 2.48 → L7 rms 6.72 (2.7배), 최대값은 20.8 까지 간다.
 *  Q15 는 ±1.0 밖에 못 담는다. 그래서 처음에는 층마다 >>1 로 깎았는데,
 *  그러면 층 간 성장이 사라진다 — 그 성장이 곧 정보다. 출력이 비었던
 *  진짜 원인이 이것이었다.
 *
 *  해결: 값은 Q15 로 꽉 채워 쓰고(정밀도 최대), 실제 크기는 지수가
 *        따로 기억한다. 벡터 하나가 지수 하나를 공유하므로
 *        "블록 부동소수점" 이라 부른다.
 *
 *        진짜 값 = q15 값 / 32768 * 2^exp
 *
 *  68000 비용: 벡터당 int8 지수 하나. 곱셈은 그대로 MULS 한 번.
 *              지수는 더하기·시프트로만 다루므로 사실상 공짜다.
 *              (FPU 없이 넓은 범위를 얻는 고전적인 방법이다) */
typedef struct {
    q15_t *v;      /* 값 (Q15) */
    int    n;       /* 개수 */
    int    exp;     /* 공유 지수. 진짜 값 = v[i]/32768 * 2^exp */
} DuskVec;

int32_t dusk_mac_row(const int8_t *w, const q15_t *x, int n);
void    dusk_quant(const q31_t *in, q15_t *out, int n, int32_t *shift_out);
void    dusk_layernorm_q(q15_t *x, const int8_t *g, const int8_t *b, int n,
                         int32_t gs, int32_t bs);
void    dusk_gelu_q(q15_t *x, int n);
void    dusk_softmax_q(q15_t *x, int n);
void    dusk_softmax_q32(int32_t *x, int n);   /* 클램프 없는 int32 점수용 */

/* ── RACHEL 추론 (dusk_rachel.c) ────────────────────────── */
int  rachel_load(const char *path);
int  rachel_init(int nodes, const char *chip, double mhz, const char *topo);
void rachel_reset(void);
int  rachel_run(const char *prompt, char *out, int outmax, long long *cycles);
int  rachel_vocab(void);
int  rachel_dim(void);
double rachel_seconds(void);      /* 마지막 실행이 68000 에서 걸릴 시간 */

int  dusk_norm(const q31_t *in, q15_t *out, int n, int in_exp);
int  dusk_vadd(q15_t *a, int a_exp, const q15_t *b, int b_exp, int n);

int  dusk_claim_port(int port);   /* 포트 자리 먼저 잡기 (dusk_http.c) */
int  dusk_serve(int port, const char *root);

#endif /* DUSK_H */
