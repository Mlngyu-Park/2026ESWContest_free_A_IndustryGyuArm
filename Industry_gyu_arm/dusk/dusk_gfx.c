/* ============================================================
 *  DUSK 렌더러 어댑터 — pl68k.c 를 라이브러리로 쓴다
 *
 *  pl68k.c 는 원래 혼자 도는 프로그램이라 main() 과 표준입출력
 *  루프를 갖고 있다. 여기서는 그 계산 부분만 가져다 쓰고,
 *  화면 한 장을 메모리로 돌려주는 함수로 감싼다.
 *
 *  pl68k.c 를 고치지 않는다 — #include 로 통째로 넣되
 *  main 과 serve_loop 는 매크로로 이름을 바꿔 비활성화한다.
 *  (원본을 건드리면 m68k_array 쪽과 어긋난다)
 * ============================================================ */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* pl68k.c 가 M_PI 를 쓰는데, 엄격한 표준 모드에서는 math.h 가
   그걸 안 준다. 원본을 고치지 않으려고 여기서 먼저 정의한다. */
#define _USE_MATH_DEFINES
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* pl68k.c 의 main/serve_loop 를 다른 이름으로 밀어낸다 */
#define main        pl68k_unused_main
#define serve_loop  pl68k_unused_serve

#include "gfx/pl68k.c"

#undef main
#undef serve_loop

/* 블록 실행 결과 (dusk_exec.c) */
extern int           dusk_exec_frames(void);
extern const double *dusk_exec_frame(int i);

/* ── 바깥에 내주는 API ─────────────────────────────────── */

static char  g_scene_json[65536];
static int   g_ready = 0;

/* 우클릭으로 옮긴 양 (화면 화소). 그린 뒤 화면을 통째로 민다 —
   투영을 건드리면 화면맞춤(자동 확대)과 싸우게 되어, 옮기는 중에
   모델 크기가 들썩인다. 다 그린 그림을 미는 게 조용하다. */
static int g_panx = 0, g_pany = 0;
void dusk_gfx_set_pan(int px, int py) { g_panx = px; g_pany = py; }

int dusk_gfx_width(void)  { return W; }
int dusk_gfx_height(void) { return H; }

void dusk_set_nodes(int n)
{
    if (n >= 1 && n <= MAXNODE) g_nodes = n;
}

int dusk_get_nodes(void) { return g_nodes; }

/* 장면 파일과 그 설명(scene.json)을 읽어둔다.
   scene.json 은 뷰어가 팔레트를 만들 때 쓴다 — 여기서는
   내용을 해석하지 않고 그대로 넘겨준다. */
/* 장면 파일 경로 — dusk_hints.c 가 좌표를 읽으려고 쓴다 */
static char g_scene_path[1024];
const char *dusk_scene_path(void) { return g_scene_path[0] ? g_scene_path : NULL; }

/* 로봇 조각을 관절에 묶을지 (장면 파일에서는 쓰지 않는다) */
void dusk_bind_joints(int on) { g_bind_joints = on ? 1 : 0; }

/* ── 조각별 자세 행렬 만들기 ──────────────────────────────
   장면의 로봇 조각들은 q=0 으로 구워져 있다 (scene.json 의 link_m
   회전부분이 전부 단위행렬인 것으로 확인).

   그래서 링크 L 에 넣을 행렬은 "q=0 에서 지금 자세로 가는 변화량":

       M_L = A_L(q)                      (q=0 에서 A_L = 단위행렬)

   여기서 A_L 은 관절 1..L 을 차례로 그 원점에서 돌린 것을 누적한
   값이다. 앱이 보내주던 것과 같은 형태(3x4, 이동은 0.1mm)다. */
void dusk_set_link_poses(const double q[6])
{
    /* 관절 원점 (m) — pl68k.c 의 JOINT_XYZ 와 같은 값, 여기선 미터 */
    static const double JO[6][3] = {
        { 0.240050, -0.234800, 0.092400 },
        { 0.258250, -0.204200, 0.141600 },
        { 0.270150, -0.176200, 0.254170 },
        { 0.267250, -0.041300, 0.259370 },
        { 0.240150,  0.017800, 0.259370 },
        { 0.260350,  0.042200, 0.236270 },
    };
    static const int AX[6] = { 2, 0, 0, 0, 1, 2 };   /* 0=X 1=Y 2=Z */

    double A[16];                       /* 누적 (4x4 행 우선) */
    memset(A, 0, sizeof A);
    A[0] = A[5] = A[10] = A[15] = 1.0;

    /* 조각 0 = base_link 는 안 움직인다 */
    double I[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};
    dusk_gfx_set_pose(0, I);

    for (int L = 1; L <= 6; L++) {
        double c = cos(q[L-1]), sn = sin(q[L-1]);
        double R[16]; memset(R,0,sizeof R);
        R[15]=1.0;
        switch (AX[L-1]) {
            case 0: R[0]=1; R[5]=c; R[6]=-sn; R[9]=sn;  R[10]=c; break;
            case 1: R[0]=c; R[2]=sn; R[5]=1;  R[8]=-sn; R[10]=c; break;
            default:R[0]=c; R[1]=-sn; R[4]=sn; R[5]=c;  R[10]=1; break;
        }
        /* 관절 원점으로 옮겼다가 돌리고 되돌린다 */
        const double *o = JO[L-1];
        double T[16];
        for (int i = 0; i < 16; i++) T[i] = R[i];
        /* T = Trans(o) * R * Trans(-o) 의 이동 성분 */
        T[3]  = o[0] - (R[0]*o[0] + R[1]*o[1] + R[2]*o[2]);
        T[7]  = o[1] - (R[4]*o[0] + R[5]*o[1] + R[6]*o[2]);
        T[11] = o[2] - (R[8]*o[0] + R[9]*o[1] + R[10]*o[2]);

        /* A = A * T */
        double N[16];
        for (int r = 0; r < 4; r++)
            for (int cc = 0; cc < 4; cc++) {
                double sacc = 0;
                for (int k = 0; k < 4; k++) sacc += A[r*4+k] * T[k*4+cc];
                N[r*4+cc] = sacc;
            }
        memcpy(A, N, sizeof A);

        /* ★ 좌표계를 맞춘다 — 이걸 빼먹어서 팔이 흩어졌다.
             기구학은 z-up 인데 장면 메쉬는 y-up 으로 저장돼 있다.
             그래서 같은 좌표계로 옮겨 곱해야 한다:  D' = Zup · D · Zup⁻¹
             (m68k_server.py 가 앱에서 하던 것과 같은 변환) */
        static const double ZUP[16]  = {1,0,0,0,  0,0,1,0,  0,-1,0,0,  0,0,0,1};
        static const double ZUPI[16] = {1,0,0,0,  0,0,-1,0, 0,1,0,0,  0,0,0,1};
        double t1[16], D[16];
        for (int r = 0; r < 4; r++)
            for (int cc = 0; cc < 4; cc++) {
                double sa = 0;
                for (int k = 0; k < 4; k++) sa += ZUP[r*4+k] * A[k*4+cc];
                t1[r*4+cc] = sa;
            }
        for (int r = 0; r < 4; r++)
            for (int cc = 0; cc < 4; cc++) {
                double sa = 0;
                for (int k = 0; k < 4; k++) sa += t1[r*4+k] * ZUPI[k*4+cc];
                D[r*4+cc] = sa;
            }

        /* 3x4 로 잘라 넣는다. 이동 단위는 0.1mm (앱과 같은 약속) */
        double m[12];
        for (int r = 0; r < 3; r++) {
            m[r*4+0] = D[r*4+0];
            m[r*4+1] = D[r*4+1];
            m[r*4+2] = D[r*4+2];
            m[r*4+3] = D[r*4+3] * 10000.0;   /* m -> 0.1mm */
        }
        dusk_gfx_set_pose(L, m);
    }
}

int dusk_gfx_init(const char *scene_path)
{
    snprintf(g_scene_path, sizeof g_scene_path, "%s", scene_path ? scene_path : "");
    /* 사인 테이블부터 만든다 — 이걸 빼먹으면 모든 회전이 0 이 되어
       삼각형이 한 점으로 뭉개지고 화면이 새까맣게 나온다.
       (pl68k.c 의 main 은 이걸 먼저 부른다) */
    init_trig();

    if (!load_mesh(scene_path)) {
        fprintf(stderr, "장면을 못 읽었습니다: %s\n", scene_path);
        return 0;
    }

    /* 같은 폴더의 scene.json 을 통째로 읽어둔다 */
    char jp[1024];
    snprintf(jp, sizeof jp, "%s", scene_path);
    char *dot = strrchr(jp, '.');
    if (dot) strcpy(dot, ".json");
    FILE *f = fopen(jp, "rb");
    if (f) {
        size_t n = fread(g_scene_json, 1, sizeof g_scene_json - 1, f);
        g_scene_json[n] = 0;
        fclose(f);
    } else {
        snprintf(g_scene_json, sizeof g_scene_json, "{\"parts\":[],\"tri\":0}");
    }

    int tf = 0;
    for (int i = 0; i < nlinks; i++) tf += links[i].nf;
    printf("  장면: 조각 %d · 삼각형 %d · 화면 %dx%d\n", nlinks, tf, W, H);

    g_ready = 1;
    return 1;
}

const char *dusk_gfx_scene_json(void) { return g_scene_json; }

/* 화면 한 장. out 은 W*H 바이트(팔레트 인덱스).
   cycles 에는 '가장 느린 노드' 의 68000 사이클이 담긴다 —
   전체를 노드 수로 나눈 값이 아니다. 모든 노드가 끝나야
   화면이 나오므로 그게 실제 걸리는 시간이다. */
int dusk_gfx_frame(double yaw, double pitch, double zoom,
                   double phase, unsigned char *out, long long *cycles)
{
    if (!g_ready || !out) return 0;

    g_yaw  = ((int)yaw % 360 + 360) % 360;
    g_pit  = (int)pitch;
    if (g_pit >  89) g_pit =  89;
    if (g_pit < -89) g_pit = -89;
    g_zoom = (int)zoom;
    if (g_zoom <  40) g_zoom =  40;
    if (g_zoom > 600) g_zoom = 600;

    /* ── 관절 각도 ─────────────────────────────────────────
       예전에는 사인파로 흔들기만 했다 (아래 옛 코드). 그래서
       블록을 만들어도 팔이 제 갈 길로 안 가고 춤만 췄다.
       이제 블록 실행이 계산한 자세를 그대로 쓴다.

       phase 는 "몇 번째 자세" 로 해석한다. 실행 결과가 없으면
       예전처럼 흔든다 (구경만 할 때 화면이 멈춰 있지 않게). */
    int ang[6];
    const double *q = NULL;
    int nf = dusk_exec_frames();
    if (nf > 0) {
        int i = (int)phase;
        if (i < 0) i = 0;
        if (i >= nf) i = nf - 1;
        q = dusk_exec_frame(i);
    }
    if (q) {
        /* ★ 관절각을 ang[] 로 주면 안 된다.
             장면 파일의 조각들은 '조립된 절대 좌표' 라서, 부모 회전을
             다시 먹이면 자식이 엉뚱한 축으로 돌아 팔이 흩어진다
             (실측: 링크들이 공중에 떠서 따로 놀았다).
             앱이 하던 방식대로 '조각별 자세 행렬' 을 넣는다. */
        for (int j = 0; j < 6; j++) ang[j] = 0;
        dusk_set_link_poses(q);
    } else {
        dusk_gfx_clear_pose();
        for (int j = 0; j < 6; j++)
            ang[j] = (int)(40 * sin(phase + j * 0.9));
    }

    int dr = 0, cu = 0;
    draw_frame(ang, &dr, &cu);

    if (g_panx || g_pany) {
        /* 화면 크기(320x240)와 보이는 크기(캔버스)가 다르므로
           대충 비율을 맞춘다 — 캔버스가 대개 2배 정도다. */
        int dx = g_panx / 2, dy = g_pany / 2;
        memset(out, 0, (size_t)(W * H));
        for (int y = 0; y < H; y++) {
            int sy = y - dy;
            if (sy < 0 || sy >= H) continue;
            for (int x = 0; x < W; x++) {
                int sx = x - dx;
                if (sx < 0 || sx >= W) continue;
                out[y*W + x] = fb[sy*W + sx];
            }
        }
    } else {
        memcpy(out, fb, (size_t)(W * H));
    }
    if (cycles) *cycles = frame_cycles();
    return 1;
}

/* 앱이 보내주는 '지금 로봇 자세' 를 조각에 적용한다.
   m 은 3x4 (회전 9 + 이동 3), 이동 단위는 0.1mm.
   앱이 꺼져 있으면 아무도 안 부르고, 마지막 자세로 계속 그린다. */
int dusk_gfx_set_pose(int part, const double *m)
{
    if (part < 0 || part >= MAXPART || !m) return 0;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++)
            part_xf[part].m[r * 4 + c] = (int32_t)(m[r * 4 + c] * 65536.0);
        part_xf[part].m[r * 4 + 3] = (int32_t)(m[r * 4 + 3] * 65536.0);
    }
    part_xf_on[part] = 1;
    return 1;
}

void dusk_gfx_clear_pose(void)
{
    for (int i = 0; i < MAXPART; i++) part_xf_on[i] = 0;
}
