/* ============================================================
 *  블록 실행 — 블록 목록을 관절 움직임으로 바꾼다
 *
 *  앱(arm_studio_web.py)의 do_grasp / do_release / goto_move 를
 *  그대로 옮긴다. ★ 규칙을 새로 만들지 않는다.
 *
 *  고정 치수 (사용자가 2026-08-14 에 정한 값, 앱과 같아야 한다)
 *    원점(판 바닥) → 도킹 윗면        115mm
 *    도킹 윗면    → 그리퍼 닿는 면    +10mm   GRASP_FACE_DZ
 *    그 면        → cr_7 (그리퍼 기준) +37mm   GRASP_DZ_DEFAULT
 *    ─────────────────────────────────────────
 *    도킹에 꽂힌 펜을 잡는 높이 = 115+10+37 = 162mm  (고정)
 *
 *    접근/상승 여유 = 60mm                     GRASP_LIFT
 *
 *  동작 정의 (사용자 정의, 앱 주석 그대로)
 *    잡기 = 열기 → 하강 → 닫기 → 상승
 *    놓기 = 하강 → 열기 → 상승 → 닫기
 *    이동 = 뽑기(수직 상승) → 수평 → 하강
 *
 *  ★ 잡기·놓기는 좌표를 갖지 않는다.
 *    좌표를 가진 블록은 [이동] 하나뿐이다. 잡기가 대상 XY 를 따로
 *    계산하면 이동과 두 개가 되어 서로 어긋난다 (앱 실측 23mm).
 *    그래서 지금 서 있는 자리에서 수직으로만 움직인다.
 * ============================================================ */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "dusk.h"

extern void dusk_fk_tool(const double q[6], double out[3]);
extern int  dusk_ik_tool(const double tgt[3], const double seed[6],
                         double j5, double j6, double out[6]);
extern const char *dusk_block_item(int i);
extern int  dusk_blocks_count(void);
extern int  dusk_dock_xyz(const char *dock, double out[3]);
extern int  dusk_pen_xyz(double out[3]);

/* ── 고정 치수 (앱과 같은 값) ───────────────────────────── */
#define GRASP_DZ_DEFAULT  0.037
#define GRASP_FACE_DZ     0.010
#define GRASP_LIFT        0.060
#define PLATE_TOP         0.020
#define PEN_H             0.139459      /* ppap 높이 (앱 실측) */

/* ── 실행 상태 ──────────────────────────────────────────── */
#define MAXFRAME 4096
static double g_frame[MAXFRAME][6];    /* 재생할 관절 자세들 */
static int    g_nframe = 0;
static double g_q[6] = {0,0,0,0,0,0};  /* 지금 자세 */
static int    g_held = 0;              /* 펜을 들고 있나 */
static char   g_err[256];

static void push(const double q[6])
{
    if (g_nframe >= MAXFRAME) return;
    memcpy(g_frame[g_nframe], q, 6*sizeof(double));
    g_nframe++;
}

int    dusk_exec_frames(void) { return g_nframe; }
const double *dusk_exec_frame(int i)
{
    return (i >= 0 && i < g_nframe) ? g_frame[i] : NULL;
}
const char *dusk_exec_error(void) { return g_err[0] ? g_err : NULL; }

/* ── 수직 이동 (앱의 _descend) ────────────────────────────
   x,y 는 그대로 두고 z 만 바꾼다. 5mm 마다 IK 를 새로 푼다 —
   중간마다 풀어야 옆으로 새지 않는다 (앱도 그렇게 한다). */
static int descend(double x, double y, double z_hi, double z_lo, const char *tag)
{
    int n = (int)(fabs(z_hi - z_lo) / 0.005);
    if (n < 6) n = 6;
    for (int k = 1; k <= n; k++) {
        double z = z_hi + (z_lo - z_hi) * (double)k / n;
        double t[3] = { x, y, z }, out[6];
        if (!dusk_ik_tool(t, g_q, g_q[4], g_q[5], out)) {
            snprintf(g_err, sizeof g_err,
                     "%s: (%.0f,%.0f) z=%.0fmm 도달 불가",
                     tag, x*1000, y*1000, z*1000);
            return 0;
        }
        out[4] = g_q[4];              /* 물컵 규칙: j5 안 건드림 */
        out[5] = g_q[5];              /* 그리퍼 상태 유지 */
        memcpy(g_q, out, sizeof g_q);
        push(g_q);
    }
    return 1;
}

/* ── 수평 이동 (앱의 _brick_move_to) ──────────────────────
   높이는 그대로 두고 xy 만 옮긴다. 관절 공간에서 부드럽게 잇는다. */
static int move_to(double x, double y, double z, const char *tag)
{
    double t[3] = { x, y, z }, goal[6];
    if (!dusk_ik_tool(t, g_q, g_q[4], g_q[5], goal)) {
        snprintf(g_err, sizeof g_err,
                 "%s: (%.0f,%.0f) z=%.0fmm 도달 불가",
                 tag, x*1000, y*1000, z*1000);
        return 0;
    }
    goal[4] = g_q[4]; goal[5] = g_q[5];

    /* 관절 보간 — 앱의 plan_joint_seq 와 같은 부드럽기 곡선 */
    double from[6]; memcpy(from, g_q, sizeof from);
    const int N = 40;
    for (int k = 1; k <= N; k++) {
        double t0 = (double)k / N;
        double e = t0*t0*(3.0 - 2.0*t0);      /* smoothstep */
        double f[6];
        for (int j = 0; j < 6; j++) f[j] = from[j] + (goal[j]-from[j]) * e;
        memcpy(g_q, f, sizeof g_q);
        push(g_q);
    }
    return 1;
}

/* ── 목표 접근점 (앱의 goto_point) ────────────────────────
   도킹  : 소켓 + 10 + 37 + 60 = 윗면에서 107mm 위
   펜    : 잡는 점 + 60, 단 펜 꼭대기보다 20mm 는 높게 */
static int goto_point(const char *oid, double out[3])
{
    double p[3];
    if (!strcmp(oid, "ppap")) {
        if (!dusk_pen_xyz(p)) {
            snprintf(g_err, sizeof g_err, "펜을 찾을 수 없습니다");
            return 0;
        }
        double grasp_z = p[2];
        double top     = p[2] + PEN_H;
        out[0]=p[0]; out[1]=p[1];
        out[2] = grasp_z + GRASP_LIFT;
        if (top + 0.02 > out[2]) out[2] = top + 0.02;
        return 1;
    }
    if (!dusk_dock_xyz(oid, p)) {
        snprintf(g_err, sizeof g_err, "도킹을 찾을 수 없습니다: %s", oid);
        return 0;
    }
    out[0]=p[0]; out[1]=p[1];
    out[2]=p[2] + GRASP_FACE_DZ + GRASP_DZ_DEFAULT + GRASP_LIFT;
    return 1;
}

/* ── 블록 하나 실행 ─────────────────────────────────────── */
static int run_one(const char *item)
{
    double now[3];
    dusk_fk_tool(g_q, now);

    /* 이동: 뽑기 → 수평 → 하강 */
    if (!strncmp(item, "@goto:", 6)) {
        double p[3];
        if (!goto_point(item + 6, p)) return 0;

        /* ① 뽑기 — 지금 자리에서 이동 높이까지 수직으로 */
        double ztr = p[2] > now[2] ? p[2] : now[2];
        if (ztr > now[2] + 1e-4 && !descend(now[0], now[1], now[2], ztr, "이동① 뽑기"))
            return 0;
        /* ② 수평 — 높이 유지한 채 목표 위로 */
        if (!move_to(p[0], p[1], ztr, "이동② 수평")) return 0;
        /* ③ 하강 — 접근 높이로 */
        if (ztr > p[2] + 1e-4 && !descend(p[0], p[1], ztr, p[2], "이동③ 하강"))
            return 0;
        return 1;
    }

    /* 잡기 = 열기 → 하강 → 닫기 → 상승 (좌표 없음: 제자리 수직) */
    if (!strncmp(item, "@grasp:", 7)) {
        double pen[3];
        if (!dusk_pen_xyz(pen)) {
            snprintf(g_err, sizeof g_err, "펜이 배치되지 않았습니다");
            return 0;
        }
        double zg   = pen[2];                       /* 잡는 높이 */
        double top  = pen[2] + PEN_H;
        double z_up = zg + GRASP_LIFT;
        if (top + 0.02 > z_up) z_up = top + 0.02;

        if (!descend(now[0], now[1], now[2], zg, "잡기② 하강")) return 0;
        g_held = 1;
        if (!descend(now[0], now[1], zg, z_up, "잡기④ 상승"))   return 0;
        return 1;
    }

    /* 놓기 = 하강 → 열기 → 상승 → 닫기 */
    if (!strncmp(item, "@release:", 9)) {
        if (!g_held) {
            snprintf(g_err, sizeof g_err, "들고 있는 물체가 없습니다 (먼저 [잡기])");
            return 0;
        }
        double d[3];
        if (!dusk_dock_xyz(item + 9, d)) {
            snprintf(g_err, sizeof g_err, "도킹을 찾을 수 없습니다: %s", item + 9);
            return 0;
        }
        double zg   = d[2] + GRASP_FACE_DZ + GRASP_DZ_DEFAULT;   /* 162mm 고정식 */
        double z_up = zg + GRASP_LIFT;

        if (!descend(now[0], now[1], now[2], zg, "놓기① 하강")) return 0;
        g_held = 0;
        if (!descend(now[0], now[1], zg, z_up, "놓기③ 상승"))   return 0;
        return 1;
    }

    /* 하강/상승 — 제자리 수직 */
    if (!strcmp(item, "@down")) {
        double tz = PLATE_TOP + GRASP_DZ_DEFAULT;
        if (tz >= now[2] - 1e-4) return 1;          /* 이미 낮다 */
        return descend(now[0], now[1], now[2], tz, "하강");
    }
    if (!strcmp(item, "@up")) {
        double tz = now[2] + GRASP_LIFT;
        return descend(now[0], now[1], now[2], tz, "상승");
    }

    /* 그리퍼 — 팔은 안 움직인다 (자세를 한 장 더 넣어 시간만 표시) */
    if (!strcmp(item, "@open") || !strcmp(item, "@close")) {
        push(g_q);
        return 1;
    }

    return 1;   /* 모르는 블록은 조용히 넘어간다 */
}

/* ── 전부 실행 ──────────────────────────────────────────── */
int dusk_exec_run(void)
{
    g_nframe = 0; g_held = 0; g_err[0] = 0;
    memset(g_q, 0, sizeof g_q);
    push(g_q);

    int n = dusk_blocks_count();
    if (n <= 0) {
        snprintf(g_err, sizeof g_err, "블록이 없습니다");
        return 0;
    }
    for (int i = 0; i < n; i++) {
        const char *it = dusk_block_item(i);
        if (!it || !it[0]) continue;
        if (!run_one(it)) return 0;
    }
    return 1;
}
