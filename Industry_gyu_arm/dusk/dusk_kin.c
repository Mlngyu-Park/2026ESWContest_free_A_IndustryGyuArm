/* ============================================================
 *  로봇 계산 — 관절각 ↔ 좌표 (start_pro 와 같은 답을 내야 한다)
 *
 *  왜 C 로 옮기나
 *    dusk 는 파이썬 0줄이 원칙이다. 그런데 블록을 실제로 돌리려면
 *    "빨간 도킹 위로 가라" -> 관절 6개 각도 를 계산해야 한다.
 *    그 계산이 앱(arm_studio_web.py)에 numpy/scipy/ikpy 로 들어 있다.
 *
 *  옮길 때 지킨 것
 *    ★ 규칙을 새로 만들지 않는다. 앱이 내는 답과 같아야 한다.
 *      (검증: scratchpad/ik_truth.json 의 29건과 대조)
 *
 *  로봇 구조 (URDF 에서 읽은 실측값, 전부 rpy=0 이라 회전 없음)
 *    j1  (0.240050, -0.234800, 0.092400)  축 Z
 *    j2  (0.018200,  0.030600, 0.049200)  축 X
 *    j3  (0.011900,  0.028000, 0.112570)  축 X
 *    j4  (-0.002900, 0.134900, 0.005200)  축 X
 *    j5  (-0.027100, 0.059100, 0.000000)  축 Y
 *    j6  (0.020200,  0.024400, -0.023100) 축 Z
 *    툴  j6 프레임에서 (-0.009, 0.0395, 0.03933513)
 *
 *  물컵 규칙 (이 프로젝트의 핵심 규칙)
 *    j2·j3·j4 는 전부 같은 X 축이라 그 합이 손목 기울기가 된다.
 *    합이 0 이면 그리퍼가 수평 — 물컵을 안 쏟는다.
 *    그래서 IK 는 j2,j3,j4 를 풀되 j4 = -(j2+j3) 으로 묶는다.
 *    j5 는 아예 안 건드린다 (앱도 그렇게 한다: _chain_level).
 * ============================================================ */
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "dusk.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── 로봇 치수 ──────────────────────────────────────────── */
typedef struct { double xyz[3]; int axis; } Joint;   /* axis 0=X 1=Y 2=Z */

static const Joint JOINT[6] = {
    {{ 0.240050000000002, -0.2348,              0.0924            }, 2},
    {{ 0.0182000000000001, 0.0306,              0.0491999999999997}, 0},
    {{ 0.0119,             0.028,               0.112569978235762 }, 0},
    {{-0.00290000000000018,0.134899814673214,   0.00519999999999987}, 0},
    {{-0.0270999999999996, 0.0591000000000002,  0.0               }, 1},
    {{ 0.0201999999999996, 0.0244000000000003, -0.0231            }, 2},
};

static const double TOOL_OFF[3] = { -0.009, 0.0395, 0.0393351262037889 };

/* 관절 한계 (모터 보정에서 나온 실측값, 라디안) */
static const double JLO[6] = { -2.152175, -1.837709, -1.598408, -1.741068, -0.127320, -2.244214 };
static const double JHI[6] = {  2.077010,  1.843845,  1.862253,  1.761010,  3.423845,  0.052155 };

void dusk_joint_limits(int k, double *lo, double *hi)
{
    if (k < 0 || k > 5) { *lo = *hi = 0; return; }
    *lo = JLO[k]; *hi = JHI[k];
}

/* ── 4x4 행렬 (행 우선) ─────────────────────────────────── */
static void mat_ident(double m[16])
{
    memset(m, 0, 16 * sizeof(double));
    m[0] = m[5] = m[10] = m[15] = 1.0;
}

static void mat_mul(const double a[16], const double b[16], double o[16])
{
    double t[16];
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            double s = 0;
            for (int k = 0; k < 4; k++) s += a[r*4+k] * b[k*4+c];
            t[r*4+c] = s;
        }
    memcpy(o, t, sizeof t);
}

/* 축 회전 + 이동 */
static void joint_mat(const Joint *j, double q, double m[16])
{
    double c = cos(q), s = sin(q);
    mat_ident(m);
    switch (j->axis) {
        case 0: m[5]=c; m[6]=-s; m[9]=s;  m[10]=c; break;   /* X */
        case 1: m[0]=c; m[2]=s;  m[8]=-s; m[10]=c; break;   /* Y */
        default:m[0]=c; m[1]=-s; m[4]=s;  m[5]=c;  break;   /* Z */
    }
    m[3] = j->xyz[0]; m[7] = j->xyz[1]; m[11] = j->xyz[2];
}

/* ── 앞방향 계산: 관절각 -> 툴 중심 좌표 ──────────────────
   앱의 tool_pos 와 같다.
   ★ 앱은 툴을 잴 때 j6(그리퍼)를 0 으로 놓는다 — 그리퍼를 벌려도
     툴 기준점이 움직이면 안 되기 때문이다. 여기서도 그렇게 한다. */
void dusk_fk_tool(const double q[6], double out[3])
{
    double T[16], m[16];
    mat_ident(T);
    for (int k = 0; k < 6; k++) {
        double qq = (k == 5) ? 0.0 : q[k];   /* j6 은 0 으로 */
        joint_mat(&JOINT[k], qq, m);
        mat_mul(T, m, T);
    }
    /* 툴 = T * TOOL_OFF */
    for (int r = 0; r < 3; r++)
        out[r] = T[r*4+0]*TOOL_OFF[0] + T[r*4+1]*TOOL_OFF[1]
               + T[r*4+2]*TOOL_OFF[2] + T[r*4+3];
}

/* ── 거꾸로 계산: 좌표 -> 관절각 ──────────────────────────
 *
 *  앱은 scipy 의 최소제곱으로 푼다. 여기서는 같은 일을
 *  자코비안 감쇠최소제곱(DLS)으로 한다 — 변수가 j1,j2,j3 셋뿐이라
 *  (j4 는 물컵 규칙으로 따라오고, j5·j6 은 고정) 아주 짧다.
 *
 *    자코비안 J = ∂(툴 위치)/∂(j1,j2,j3)     3x3
 *    Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ · 오차
 *
 *  λ(감쇠)를 두는 이유: 팔이 쭉 펴진 자세에서는 J 가 거의 납작해져
 *  그냥 역행렬을 쓰면 Δq 가 폭발한다. λ 가 그걸 눌러준다.
 */

/* 물컵 규칙을 먹인 관절 벡터 만들기 */
static void level_q(double j1, double j2, double j3,
                    double j5, double j6, double q[6])
{
    q[0]=j1; q[1]=j2; q[2]=j3;
    q[3]=-(j2+j3);            /* ★ 합 0 -> 수평 유지 */
    q[4]=j5; q[5]=j6;
}

static int in_limits(const double q[6])
{
    for (int k = 0; k < 5; k++)
        if (q[k] < JLO[k] - 1e-6 || q[k] > JHI[k] + 1e-6) return 0;
    return 1;
}

/* 3x3 역행렬 */
static int inv3(const double a[9], double o[9])
{
    double d = a[0]*(a[4]*a[8]-a[5]*a[7])
             - a[1]*(a[3]*a[8]-a[5]*a[6])
             + a[2]*(a[3]*a[7]-a[4]*a[6]);
    if (fabs(d) < 1e-18) return 0;
    double id = 1.0/d;
    o[0]= (a[4]*a[8]-a[5]*a[7])*id;  o[1]=-(a[1]*a[8]-a[2]*a[7])*id;  o[2]= (a[1]*a[5]-a[2]*a[4])*id;
    o[3]=-(a[3]*a[8]-a[5]*a[6])*id;  o[4]= (a[0]*a[8]-a[2]*a[6])*id;  o[5]=-(a[0]*a[5]-a[2]*a[3])*id;
    o[6]= (a[3]*a[7]-a[4]*a[6])*id;  o[7]=-(a[0]*a[7]-a[1]*a[6])*id;  o[8]= (a[0]*a[4]-a[1]*a[3])*id;
    return 1;
}

/* 씨앗 하나에서 풀어본다. 성공하면 1. */
static int ik_from(const double tgt[3], const double seed[6],
                   double j5, double j6, double out[6])
{
    double j1 = seed[0], j2 = seed[1], j3 = seed[2];
    double q[6], cur[3];

    for (int it = 0; it < 120; it++) {
        level_q(j1, j2, j3, j5, j6, q);
        dusk_fk_tool(q, cur);

        double e[3] = { tgt[0]-cur[0], tgt[1]-cur[1], tgt[2]-cur[2] };
        double err = sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
        if (err < 1e-6) {                 /* 다 왔다 */
            if (!in_limits(q)) return 0;
            memcpy(out, q, 6*sizeof(double));
            return 1;
        }

        /* 자코비안을 수치로 (변수 3개뿐이라 이게 제일 간단하고 확실하다) */
        double J[9];
        const double h = 1e-6;
        double base[3] = { j1, j2, j3 };
        for (int c = 0; c < 3; c++) {
            double v[3] = { base[0], base[1], base[2] };
            v[c] += h;
            double qh[6], ph[3];
            level_q(v[0], v[1], v[2], j5, j6, qh);
            dusk_fk_tool(qh, ph);
            J[0*3+c] = (ph[0]-cur[0]) / h;
            J[1*3+c] = (ph[1]-cur[1]) / h;
            J[2*3+c] = (ph[2]-cur[2]) / h;
        }

        /* A = J Jᵀ + λ²I */
        double lam = 0.02, A[9];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++) {
                double s = 0;
                for (int k = 0; k < 3; k++) s += J[r*3+k]*J[c*3+k];
                A[r*3+c] = s + (r==c ? lam*lam : 0.0);
            }
        double Ai[9];
        if (!inv3(A, Ai)) return 0;

        /* Δq = Jᵀ Ai e */
        double t[3];
        for (int r = 0; r < 3; r++)
            t[r] = Ai[r*3+0]*e[0] + Ai[r*3+1]*e[1] + Ai[r*3+2]*e[2];
        double dq[3];
        for (int c = 0; c < 3; c++)
            dq[c] = J[0*3+c]*t[0] + J[1*3+c]*t[1] + J[2*3+c]*t[2];

        /* 한 발짝이 너무 크면 자른다 (튀어서 발산하는 걸 막는다) */
        double n = sqrt(dq[0]*dq[0]+dq[1]*dq[1]+dq[2]*dq[2]);
        const double STEP = 0.3;
        if (n > STEP) { double s = STEP/n; dq[0]*=s; dq[1]*=s; dq[2]*=s; }

        j1 += dq[0]; j2 += dq[1]; j3 += dq[2];
    }
    return 0;
}

/* ── 바깥에 내주는 IK ─────────────────────────────────────
   앱과 같이 씨앗을 여러 개 시도한다. 씨앗이 나쁘면 닿는 점도
   못 푸는 일이 있어서, 앱도 그래서 여러 개를 쓴다. */
int dusk_ik_tool(const double tgt[3], const double seed[6],
                 double j5, double j6, double out[6])
{
    static const double S[][3] = {
        { 0.0,  0.0,   0.0  },
        { 0.5,  0.2,  -0.7  },
        { 1.0,  0.4,  -0.7  },
        {-0.5,  0.7,  -1.1  },
        { 0.0,  0.6,  -0.9  },
        { 0.3, -0.2,   0.3  },
    };

    /* ① 지금 자세에서 출발 (제일 가까운 답이 나온다) */
    if (seed && ik_from(tgt, seed, j5, j6, out)) return 1;

    /* ② 안 되면 여러 자리에서 다시 */
    for (unsigned i = 0; i < sizeof S / sizeof S[0]; i++) {
        double sd[6] = { S[i][0], S[i][1], S[i][2], 0, j5, j6 };
        if (ik_from(tgt, sd, j5, j6, out)) return 1;
    }
    return 0;
}
