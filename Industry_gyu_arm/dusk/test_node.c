/* ============================================================
 *  정수 커널 검증 — float 원본과 얼마나 다른가
 *
 *  ★ 이게 1단계 관문이다. HANDOFF_LLM_ON_CPU.md 가 말한
 *    "여기서 정확도가 안 나오면 하드웨어를 만들어도 소용없다".
 *
 *  방법: 같은 입력을 float(C++ 원본 방식)와 정수(68000 방식)로
 *        각각 계산해 차이를 잰다. 정답은 float 쪽이다.
 * ============================================================ */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "dusk.h"

static unsigned long rng = 12345;
static int rnd(int lo, int hi) {
    rng = rng * 1103515245 + 12345;
    return lo + (int)((rng >> 16) % (unsigned)(hi - lo + 1));
}

/* float 판 GELU — rachel.cpp 와 같은 식 */
static double gelu_f(double v) {
    return 0.5*v*(1.0 + tanh(0.7978845608*(v + 0.044715*v*v*v)));
}

int main(void) {
    printf("=== DUSK 정수 커널 검증 (float 원본 대조) ===\n\n");

    /* ── 1. MAC ────────────────────────────────────────── */
    {
        enum { N = 320 };
        int8_t w[N]; q15_t x[N]; double xf[N];
        for (int i = 0; i < N; i++) {
            w[i]  = (int8_t)rnd(-127, 127);
            int v = rnd(-32000, 32000);
            x[i]  = (q15_t)v;
            xf[i] = v / 32768.0;              /* 같은 값을 실수로 */
        }
        int32_t got = dusk_mac_row(w, x, N);
        double  want = 0;
        for (int i = 0; i < N; i++) want += (double)w[i] * xf[i];
        double got_f = got / 32768.0;
        double err = fabs(got_f - want) / (fabs(want) + 1e-9);
        printf("MAC 320개    정수 %.4f · 실수 %.4f · 상대오차 %.6f%%  %s\n",
               got_f, want, err*100, err < 1e-6 ? "완전일치" : "차이있음");
    }

    /* ── 2. LayerNorm ──────────────────────────────────── */
    {
        enum { N = 320 };
        q15_t x[N]; double xf[N];
        for (int i = 0; i < N; i++) {
            int v = rnd(-20000, 20000);
            x[i] = (q15_t)v; xf[i] = v / 32768.0;
        }
        dusk_layernorm_q(x, NULL, NULL, N, 128, 128);
        /* float 판 */
        double mu = 0; for (int i=0;i<N;i++) mu += xf[i]; mu /= N;
        double va = 0; for (int i=0;i<N;i++){double d=xf[i]-mu; va+=d*d;} va=va/N+1e-5;
        double inv = 1.0/sqrt(va);
        double maxerr = 0, sumerr = 0;
        for (int i = 0; i < N; i++) {
            double want = (xf[i]-mu)*inv*0.5;      /* 정수판은 16384 = 0.5 배율 */
            double got  = x[i]/32768.0;
            double e = fabs(got - want);
            if (e > maxerr) maxerr = e;
            sumerr += e;
        }
        printf("LayerNorm    최대오차 %.5f · 평균 %.5f  %s\n",
               maxerr, sumerr/N, maxerr < 0.01 ? "쓸만함" : "★ 위험");
    }

    /* ── 3. GELU ───────────────────────────────────────── */
    {
        enum { N = 256 };
        q15_t x[N]; double xf[N];
        for (int i = 0; i < N; i++) {
            int v = rnd(-32000, 32000);
            x[i] = (q15_t)v; xf[i] = v / 32768.0;
        }
        dusk_gelu_q(x, N);
        double maxerr = 0, sumerr = 0;
        for (int i = 0; i < N; i++) {
            double want = gelu_f(xf[i]);
            double got  = x[i]/32768.0;
            double e = fabs(got - want);
            if (e > maxerr) maxerr = e;
            sumerr += e;
        }
        printf("GELU         최대오차 %.5f · 평균 %.5f  %s\n",
               maxerr, sumerr/N, maxerr < 0.02 ? "쓸만함" : "★ 위험");
    }

    /* ── 4. Softmax ────────────────────────────────────── */
    {
        enum { N = 64 };
        q15_t x[N]; double xf[N];
        for (int i = 0; i < N; i++) {
            int v = rnd(-30000, 30000);
            x[i] = (q15_t)v; xf[i] = v / 32768.0;
        }
        dusk_softmax_q(x, N);
        double mx = -1e30;
        for (int i=0;i<N;i++) if (xf[i]>mx) mx=xf[i];
        double s = 0, ef[64];
        for (int i=0;i<N;i++){ ef[i]=exp(xf[i]-mx); s+=ef[i]; }
        double maxerr = 0;
        for (int i = 0; i < N; i++) {
            double want = ef[i]/s;
            double got  = x[i]/32768.0;
            double e = fabs(got - want);
            if (e > maxerr) maxerr = e;
        }
        printf("Softmax      최대오차 %.5f  %s\n",
               maxerr, maxerr < 0.02 ? "쓸만함" : "★ 위험");
    }

    printf("\n주의: 이건 커널 하나씩의 오차다. 8개 층을 거치면 누적된다.\n"
           "      진짜 관문은 200개 문장 검증이다.\n");
    return 0;
}
