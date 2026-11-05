/* ============================================================
 *  DUSK CPU 쪽 — 68000 한 개가 뭉치를 부리는 코드
 *
 *  CPU 가 하는 일은 명령을 큐에 넣는 것뿐이다. 계산은 안 한다.
 *  이 파일의 존재 이유는 "CPU 하나로 충분한가?" 를 재는 것이다.
 *
 *  ★ 3D 렌더러에서 배운 것: 공통 작업이 94% 를 먹으면
 *    노드를 32개 써도 안 빨라진다. 여기선 CPU 가 그 자리에 있다.
 *    common_cycles 와 cpu_cycles 를 따로 세는 이유다.
 * ============================================================ */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "dusk.h"

/* CPU 가 명령 하나를 발행하는 데 드는 68000 사이클.
 *   16바이트 구조체를 큐에 쓰고(MOVE.L ×4) 문 두드리기.
 *   MOVE.L 이 12T, 4번 = 48T, 포인터 갱신·경계검사 ~20T, 신호 12T */
#define T_SUBMIT   80
/* 배리어: 노드 전부가 끝났는지 확인 (노드 수에 비례) */
#define T_SYNC_PER_NODE  24

const DuskChip *dusk_find_chip(const char *name) {
    for (int i = 0; i < DUSK_NCHIPS; i++)
        if (!strcmp(DUSK_CHIPS[i].name, name)) return &DUSK_CHIPS[i];
    return NULL;
}

void dusk_init(Dusk *D, int nodes, const char *chip, double mhz, const char *topo) {
    memset(D, 0, sizeof(*D));
    if (nodes < 1) nodes = 1;
    if (nodes > DUSK_MAXNODE) nodes = DUSK_MAXNODE;
    D->nodes  = nodes;
    D->clk_hz = mhz * 1e6;
    D->chip   = dusk_find_chip(chip);
    if (!D->chip) D->chip = &DUSK_CHIPS[0];
    D->topo = !strcmp(topo,"ring") ? 0 : !strcmp(topo,"group") ? 1 : 2;
    for (int i = 0; i < nodes; i++) D->node[i].id = i;
}

/* 링 홉 수 — 위상에 따라 데이터가 몇 번 건너뛰는가 */
static int hops(const Dusk *D) {
    int N = D->nodes;
    if (D->topo == 0) return N;                       /* 링: 최악 N홉 */
    if (D->topo == 1) return 16 + (N + 15) / 16;      /* 그룹 */
    int h = 1; while ((1 << h) < N) h++;              /* 트리: log2(N)*2 */
    return h * 2 > 0 ? h * 2 : 2;
}

/* ── 명령 하나를 노드들에게 나눠 실행시킨다 ──────────────── */
void dusk_submit(Dusk *D, const DuskCmd *c) {
    D->ncmd++;
    D->cpu_cycles += T_SUBMIT;          /* ★ CPU 부하는 노드 수와 무관 */

    const DuskChip *K = D->chip;
    int N = D->nodes;
    int rows = c->rows, cols = c->cols;

    if (c->op == DUSK_SYNC) {
        D->cpu_cycles += (long long)T_SYNC_PER_NODE * N;
        return;
    }

    /* 행 분할: 노드 k 는 i ≡ k (mod N) 행을 맡는다.
     * 행이 노드보다 적으면 남는 노드는 논다 — idle 로 기록한다.
     * (vocab 167행에 256노드를 쓰면 89개가 노는 것이 여기서 잡힌다) */
    for (int k = 0; k < N; k++) {
        int my_rows = rows / N + (k < rows % N ? 1 : 0);
        long long work = 0, mac = 0;

        switch (c->op) {
        case DUSK_MATVEC:
            mac  = (long long)my_rows * cols;
            work = mac * K->t_mac;
            break;
        case DUSK_XFORM:                       /* 정점 하나 = 곱9 + 덧6 */
            mac  = (long long)my_rows * 9;
            work = (long long)my_rows * (9*K->t_mul + 6*16);
            break;
        case DUSK_RASTER:                      /* 픽셀 하나 = 깊이비교 + 쓰기 */
            work = (long long)my_rows * cols * 20;
            break;
        case DUSK_VADD:
            work = (long long)my_rows * (2*K->t_ld + 8);
            break;
        case DUSK_ACT:
            /* GELU 는 tanh 근사(곱셈 여러 번), LayerNorm 은 나눗셈+제곱근 */
            if (c->arg == ACT_GELU)      work = (long long)my_rows * (4*K->t_mul + 60);
            else if (c->arg == ACT_LAYERNORM) work = (long long)my_rows * 40 + K->t_div * 2 + 200;
            else                          work = (long long)my_rows * (K->t_div + 80);
            break;
        default: break;
        }

        D->node[k].cycles += work;
        D->node[k].macs   += mac;
        if (my_rows == 0) D->node[k].idle++;   /* 이번 명령에서 논 노드 */

        /* 통신: 입력을 받고(int16) 결과를 내보낸다(int32).
         * ★ 노드가 많아질수록 입력 뿌리기는 그대로인데 노드마다 받아야
         *   하므로 총 통신량이 늘어난다. 이걸 노드별로 세야 정직하다. */
        D->node[k].bytes_in  += (long long)cols * 2;
        D->node[k].bytes_out += (long long)my_rows * 4;
    }
}

void dusk_sync(Dusk *D) {
    DuskCmd c; memset(&c, 0, sizeof c);
    c.op = DUSK_SYNC;
    dusk_submit(D, &c);
}

/* ── 실제 걸리는 시간 ────────────────────────────────────
 *  ★ 전체÷노드수가 아니다. 가장 느린 노드가 결정한다.
 *    거기에 CPU 와 통신이 더해진다. */
double dusk_seconds(const Dusk *D) {
    long long peak = 0;
    long long ring_bytes = 0;
    for (int i = 0; i < D->nodes; i++) {
        if (D->node[i].cycles > peak) peak = D->node[i].cycles;
        ring_bytes += D->node[i].bytes_in + D->node[i].bytes_out;
    }
    double t_node = (double)(peak + D->common_cycles) / D->clk_hz;
    double t_cpu  = (double)D->cpu_cycles / D->clk_hz;
    double t_ring = (double)ring_bytes * hops(D) * D->chip->t_ring
                  / (double)D->nodes / D->clk_hz;
    /* 노드 계산과 통신은 겹칠 수 있으나 CPU 발행은 앞단이라 더한다.
     * 보수적으로: max(노드, 통신) + CPU */
    double t_par = t_node > t_ring ? t_node : t_ring;
    return t_par + t_cpu;
}

void dusk_report(const Dusk *D, const char *title) {
    long long peak = 0, sum = 0, mn = -1, macs = 0, ring = 0, idle = 0;
    for (int i = 0; i < D->nodes; i++) {
        long long c = D->node[i].cycles;
        if (c > peak) peak = c;
        if (mn < 0 || c < mn) mn = c;
        sum  += c;
        macs += D->node[i].macs;
        ring += D->node[i].bytes_in + D->node[i].bytes_out;
        idle += D->node[i].idle;
    }
    double t_total = dusk_seconds(D);
    double t_node  = (double)(peak + D->common_cycles) / D->clk_hz;
    double t_cpu   = (double)D->cpu_cycles / D->clk_hz;
    double t_ring  = (double)ring * hops(D) * D->chip->t_ring
                   / (double)D->nodes / D->clk_hz;

    printf("\n=== %s ===\n", title);
    printf("  칩 %s · %.1fMHz · 노드 %d개 · 홉 %d\n",
           D->chip->name, D->clk_hz/1e6, D->nodes, hops(D));
    printf("  명령 %lld개 · 총 MAC %lld\n", D->ncmd, macs);
    printf("  노드 사이클   최대 %lld · 최소 %lld · 평균 %lld\n",
           peak, mn, D->nodes ? sum/D->nodes : 0);
    if (peak > 0 && mn >= 0)
        printf("  부하 불균형   %.1f%% (최대/최소)\n",
               mn > 0 ? 100.0*(peak-mn)/peak : 100.0);
    printf("  ── 시간 ──\n");
    printf("  노드 계산     %8.3f초  (%.1f%%)\n", t_node, 100*t_node/t_total);
    printf("  CPU 발행      %8.3f초  (%.1f%%)  %s\n", t_cpu, 100*t_cpu/t_total,
           t_cpu > t_node*0.5 ? "★ CPU 가 병목이다" : "");
    printf("  통신          %8.3f초  (%.1f%%)\n", t_ring, 100*t_ring/t_total);
    if (D->common_cycles)
        printf("  공통(못나눔)  %8.3f초  ★ 이게 크면 노드를 늘려도 안 빨라진다\n",
               (double)D->common_cycles/D->clk_hz);
    if (idle)
        printf("  논 노드       %lld회 (행이 노드보다 적었다)\n", idle);
    printf("  ────────────────────────\n");
    printf("  합계          %8.3f초 = %.2f분\n", t_total, t_total/60);
}

/* ── 카운터 초기화 ─────────────────────────────────────────
   한 문장을 처리할 때마다 0 부터 다시 센다.
   (dusk_init 을 다시 부르면 메모리를 또 잡으므로 이것만 쓴다) */
void dusk_reset_counters(Dusk *D) {
    for (int i = 0; i < D->nodes; i++) {
        D->node[i].cycles = 0;
        D->node[i].macs = 0;
        D->node[i].bytes_in = 0;
        D->node[i].bytes_out = 0;
    }
    D->cpu_cycles = 0;
    D->common_cycles = 0;
}

/* 가장 느린 노드의 사이클 — 이게 실제 걸리는 시간을 결정한다.
   전체를 노드 수로 나눈 값이 아니다 (렌더러에서 확인한 것과 같다). */
long long dusk_peak_cycles(const Dusk *D) {
    long long peak = 0;
    for (int i = 0; i < D->nodes; i++)
        if (D->node[i].cycles > peak) peak = D->node[i].cycles;
    return peak + D->common_cycles + D->cpu_cycles;
}
