/* PL-68K — 68000 아키텍처 그대로 구현한 3D 렌더러 (PC 에서 구동)
 *
 *   PL-68000(LMP88959, public domain)이 8MHz 68000 하나로 실시간을 낸 기법을
 *   그대로 따른다. 다른 점은 '어디서 도느냐' 뿐이다:
 *     · 정수 전용 (고정소수점 16.16) — 68000 에는 FPU 가 없다
 *     · 플랫 셰이딩 (면당 색 하나) — 픽셀 보간 곱셈이 사라진다
 *     · 백페이스 컬링 — 안 보이는 면은 계산도 안 한다
 *     · 인덱스 컬러 (팔레트) — 픽셀당 1바이트
 *     · 320x240 저해상도 — 1980년대 표준
 *
 *   모든 연산에 68000 사이클을 세어, 실제 칩에서 몇 fps 인지 같이 낸다.
 *
 *   빌드: gcc -O2 -o pl68k pl68k.c -lm
 *   실행: ./pl68k robot_lo.bin [프레임수]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

#define W       320
#define H       240
#define MAXV    2048
#define MAXF    2048
#define FP      16                  /* 고정소수점 소수부 비트 */
#define ONE     (1 << FP)
static int g_fov = 200;        /* 투영 배율 — 모델 크기에 맞춰 자동 결정 */

/* ── 68000 사이클 계수 (모토로라 공식 타이밍) ───────────────── */
#define C_MULS      70              /* MULS.W  최악 */
#define C_MOVE      8               /* MOVE.W  (An),Dn */
#define C_ADD       8               /* ADD.L */
#define C_CMP       6
#define C_BRA      10
#define C_DIVS    158               /* DIVS.W  최악 */

static long long cycles;            /* 누적 68000 사이클 (전체 합) */

/* ── 68000 어레이: 화면을 가로띠로 나눠 노드마다 하나씩 맡긴다 ──
   각 노드는 자기 띠에 걸리는 삼각형만 그린다. 삼각형 목록은 모두가
   갖고 있고(브로드캐스트), 자기 띠 밖이면 그냥 건너뛴다.
   설계상 노드끼리 프레임버퍼를 공유하지 않으므로 이 방식이 맞다 —
   각자 자기 띠만 메모리에 들고 있으면 된다.

   프레임 시간 = 가장 오래 걸린 노드 (다 끝나야 화면이 나온다).
   전체를 노드 수로 나누는 게 아니다 — 삼각형이 화면에 고르게
   퍼져 있지 않아서, 바쁜 노드 하나가 전체를 잡아먹는다. */
#define MAXNODE 64
static int       g_nodes = 1;          /* 노드 수 (1이면 어레이 안 씀) */
static long long node_cyc[MAXNODE];    /* 노드별 사이클 */
static int       cur_node = -1;        /* 지금 계산 중인 노드 (-1=공통) */
#define CYC(n)  do { cycles += (n); \
                     if (cur_node >= 0) node_cyc[cur_node] += (n); } while (0)

/* ── 자료구조 ──────────────────────────────────────────────── */
typedef struct { int32_t x, y, z; } V3;      /* 고정소수점 */
typedef struct { uint16_t a, b, c; } Tri;

typedef struct {
    int nv, nf;
    V3   v[MAXV];
    Tri  f[MAXF];
} Mesh;

/* 조각마다 '지금 자세' 행렬. 앱이 로봇을 움직이면 이것만 갈아끼우면 된다 —
   메쉬를 다시 굽지 않아도 되니 훨씬 빠르다. (Mat 은 아래에서 정의) */
typedef struct { int32_t m[12]; } Mat;

#define MAXPART 64
static Mesh   links[MAXPART];
/* 조각마다 팔레트 띠를 하나씩 준다. 로봇 링크는 관절이 붙고,
   바닥·설비는 고정이다. joint[i] < 0 이면 관절 없이 그대로 그린다. */
static int    part_band[MAXPART];
static int    part_joint[MAXPART];
static int    scene_mode;      /* M68S(공장 전체) 로 읽었나 */
static Mat    part_xf[MAXPART];
static int    part_xf_on[MAXPART];
/* 조각마다 '지금 자세' 행렬. 앱이 로봇을 움직이면 이것만 갱신하면 된다 —
   메쉬를 다시 굽지 않아도 되니 훨씬 빠르다. */

static int    nlinks;
static int    g_span = 400;   /* 모델 최대폭 mm */
/* ── 카메라 (마우스가 조종한다) ────────────────────────────
   PL-68000 원본과 같은 방식: 매 프레임 입력을 읽어 시점 행렬을 다시 만든다.
   각도는 정수 도(degree) — 68000 은 부동소수점이 없다. */
static int    g_yaw  =  28;   /* 좌우 (마우스 X) */
static int    g_pit  = -18;   /* 위아래 (마우스 Y) */
static int    g_zoom = 160;   /* 거리 배율 % (휠) */
static int    g_fovv = 200;

/* URDF 관절 원점 — 부모 오프셋을 누적한 절대좌표, 0.1mm 단위.
   메쉬 정점과 같은 단위·같은 원점이다 (simplify.py 가 조립해서 저장한다).
   axis: 0=X, 1=Y, 2=Z */
#define UNIT 10                 /* 1mm = 10 (0.1mm 단위) */
static const int32_t JOINT_XYZ[6][3] = {
    {  2401,  -2348,   924},   /* j1  base_link → l1  (Z축) */
    {  2583,  -2042,  1416},   /* j2  l1 → l2  (X축) */
    {  2702,  -1762,  2542},   /* j3  l2 → l3  (X축) */
    {  2673,   -413,  2594},   /* j4  l3 → l4  (X축) */
    {  2402,    178,  2594},   /* j5  l4 → l5  (Y축) */
    {  2604,    422,  2363},   /* j6  l5 → l6  (Z축) */
};
static const int JOINT_AXIS[6] = { 2, 0, 0, 0, 1, 2 };
static int32_t g_cx, g_cy, g_cz;      /* 중심 보정량 */
static V3 g_bmn, g_bmx;               /* 중심 보정 후 경계상자 */
static int g_cull_z, g_cull_a;
static uint8_t fb[W*H];             /* 프레임버퍼 (팔레트 인덱스) */
static int32_t zb[W*H];             /* 깊이버퍼 */

/* ── 고정소수점 ────────────────────────────────────────────── */
static inline int32_t fmul(int32_t a, int32_t b) {
    CYC(C_MULS);                    /* 68000: MULS.W 두 번 + 조합 */
    return (int32_t)(((int64_t)a * b) >> FP);
}
static inline int32_t fdiv(int32_t a, int32_t b) {
    CYC(C_DIVS);
    return b ? (int32_t)(((int64_t)a << FP) / b) : 0;
}

/* 정수 사인/코사인 테이블 — 68000 에는 삼각함수가 없다 */
static int32_t sintab[256];
static void init_trig(void) {
    for (int i = 0; i < 256; i++)
        sintab[i] = (int32_t)(sin(i * 2 * M_PI / 256) * ONE);
}
#define SIN(a) sintab[(a) & 255]
#define COS(a) sintab[((a) + 64) & 255]

/* ── 4x4 대신 3x4 행렬 (마지막 행은 항상 0,0,0,1) ──────────── */


static Mat mat_ident(void) {
    Mat r = {{ONE,0,0,0, 0,ONE,0,0, 0,0,ONE,0}};
    return r;
}
/* 3x4 행렬 곱 (마지막 행 0,0,0,1 생략).
   r = a·b  —  회전부는 3x3 곱, 평행이동은 a 의 회전이 b 의 이동에 적용된 뒤
   a 의 이동이 더해진다. 이 순서를 틀리면 물체가 화면 밖으로 날아간다. */
static Mat mat_mul(Mat a, Mat b) {
    Mat r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            int32_t s = 0;
            for (int k = 0; k < 3; k++)
                s += fmul(a.m[i*4+k], b.m[k*4+j]);
            r.m[i*4+j] = s;
        }
        int32_t t = a.m[i*4+3];
        for (int k = 0; k < 3; k++)
            t += fmul(a.m[i*4+k], b.m[k*4+3]);
        CYC(C_ADD*3);
        r.m[i*4+3] = t;
    }
    return r;
}
static Mat mat_rotx(int ang) {
    int32_t s = SIN(ang), c = COS(ang);
    Mat r = {{ONE,0,0,0, 0,c,-s,0, 0,s,c,0}};
    return r;
}
static Mat mat_roty(int ang) {
    int32_t s = SIN(ang), c = COS(ang);
    Mat r = {{c,0,s,0, 0,ONE,0,0, -s,0,c,0}};
    return r;
}
static Mat mat_rotz(int ang) {
    int32_t s = SIN(ang), c = COS(ang);
    Mat r = {{c,-s,0,0, s,c,0,0, 0,0,ONE,0}};
    return r;
}
static Mat mat_trans(int32_t x, int32_t y, int32_t z) {
    Mat r = {{ONE,0,0,x, 0,ONE,0,y, 0,0,ONE,z}};
    return r;
}
static V3 xform(Mat m, V3 v) {
    V3 r;
    r.x = fmul(m.m[0],v.x) + fmul(m.m[1],v.y) + fmul(m.m[2],v.z) + m.m[3];
    r.y = fmul(m.m[4],v.x) + fmul(m.m[5],v.y) + fmul(m.m[6],v.z) + m.m[7];
    r.z = fmul(m.m[8],v.x) + fmul(m.m[9],v.y) + fmul(m.m[10],v.z) + m.m[11];
    CYC(C_ADD*6);
    return r;
}

/* ── 삼각형 채우기 (플랫 셰이딩, 인덱스 컬러) ───────────────
 *   PL-68000 방식: 곱셈 없이 스캔라인 증분만 쓴다.
 *   68000 에서 픽셀 하나 = MOVE.B + CMP(깊이) + 증분 ≈ 20 사이클
 */
static void tri_fill(int32_t x0,int32_t y0,int32_t z0, int32_t x1,int32_t y1,int32_t z1,
                     int32_t x2,int32_t y2,int32_t z2, uint8_t col) {
    /* 좌표는 16.16 고정소수점 화면좌표.
       PL-68000 과 같은 방식으로 경계상자 안을 훑으며 모서리 함수(edge function)
       부호로 안/밖을 가른다. 곱셈·덧셈만 쓰므로 68000 에서도 그대로 돈다.
       스캔라인 교점 방식과 달리 1픽셀보다 작은 면도 최소 한 점은 남는다. */
    int32_t mnx = x0 < x1 ? (x0 < x2 ? x0 : x2) : (x1 < x2 ? x1 : x2);
    int32_t mxx = x0 > x1 ? (x0 > x2 ? x0 : x2) : (x1 > x2 ? x1 : x2);
    int32_t mny = y0 < y1 ? (y0 < y2 ? y0 : y2) : (y1 < y2 ? y1 : y2);
    int32_t mxy = y0 > y1 ? (y0 > y2 ? y0 : y2) : (y1 > y2 ? y1 : y2);

    int px0 = (int)(mnx >> FP), px1 = (int)(mxx >> FP);
    int py0 = (int)(mny >> FP), py1 = (int)(mxy >> FP);
    if (px1 < px0) px1 = px0;
    if (py1 < py0) py1 = py0;
    if (px0 < 0) px0 = 0;  if (px1 >= W) px1 = W - 1;
    if (py0 < 0) py0 = 0;  if (py1 >= H) py1 = H - 1;
    if (px0 > px1 || py0 > py1) return;

    /* 전체 면적 (부호 포함). 0 이면 선분이다 — 그래도 한 점은 찍는다. */
    int64_t area = (int64_t)(x1-x0)*(y2-y0) - (int64_t)(y1-y0)*(x2-x0);
    int flip = area < 0;
    if (area == 0) {
        int cx = (int)(((mnx + mxx) / 2) >> FP), cy = (int)(((mny + mxy) / 2) >> FP);
        if (cx >= 0 && cx < W && cy >= 0 && cy < H) {
            int32_t z = z0;
            CYC(20);
            if (z < zb[cy*W+cx]) { zb[cy*W+cx] = z; fb[cy*W+cx] = col; }
        }
        return;
    }

    /* 모서리 함수는 x,y 에 대해 1차식이다 — 한 픽셀 옆으로 가면
       일정한 값만 더해진다. 그래서 곱셈은 행 시작에서만 하고
       안쪽 루프는 덧셈만 쓴다. 68000 에서 MULS(70) 를 ADD(8) 로 바꾸는
       고전적인 최적화다. */
    int32_t A0 = -(y2-y1), B0 = (x2-x1);
    int32_t A1 = -(y0-y2), B1 = (x0-x2);
    int32_t A2 = -(y1-y0), B2 = (x1-x0);
    int hit = 0;
    for (int y = py0; y <= py1; y++) {
        /* 이 줄을 맡은 노드로 전환한다 — 화면을 가로띠로 32등분했고
           노드마다 자기 띠만 그린다. 픽셀·나눗셈 비용이 그 노드에 쌓인다. */
        if (g_nodes > 1) cur_node = (y * g_nodes) / H;
        int32_t py = ((int32_t)y << FP) + (1 << (FP-1));   /* 픽셀 중심 */
        int32_t pxs = ((int32_t)px0 << FP) + (1 << (FP-1));
        /* 행 첫 픽셀에서만 곱셈으로 값을 세운다 */
        int64_t w0 = (int64_t)B0*(py-y1) + (int64_t)A0*(pxs-x1);
        int64_t w1 = (int64_t)B1*(py-y2) + (int64_t)A1*(pxs-x2);
        int64_t w2 = (int64_t)B2*(py-y0) + (int64_t)A2*(pxs-x0);
        CYC(C_MULS*6 + C_ADD*6);
        /* 한 픽셀 오른쪽으로 갈 때 더할 양 */
        int64_t s0 = (int64_t)A0 << FP, s1 = (int64_t)A1 << FP, s2 = (int64_t)A2 << FP;
        if (flip) { w0=-w0; w1=-w1; w2=-w2; s0=-s0; s1=-s1; s2=-s2; }
        uint8_t *row  = fb + y*W;
        int32_t *zrow = zb + y*W;
        int64_t aa = flip ? -area : area;
        for (int x = px0; x <= px1; x++, w0 += s0, w1 += s1, w2 += s2) {
            CYC(C_ADD*3);                     /* 모서리 함수 갱신 = 덧셈 3번 */
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;
            int32_t z = (int32_t)(((__int128)w0*z0 + (__int128)w1*z1
                                 + (__int128)w2*z2) / aa);
            CYC(20);                          /* 픽셀 1개: 깊이 비교 + MOVE.B */
            hit = 1;
            if (z < zrow[x]) { zrow[x] = z; row[x] = col; }
        }
    }

    /* 너무 얇아 픽셀 중심을 하나도 못 덮은 면 → 중심에 한 점 */
    if (!hit) {
        int cx = (int)(((mnx + mxx) / 2) >> FP), cy = (int)(((mny + mxy) / 2) >> FP);
        if (cx >= 0 && cx < W && cy >= 0 && cy < H) {
            int32_t z = z0;
            if (z < zb[cy*W+cx]) { zb[cy*W+cx] = z; fb[cy*W+cx] = col; }
        }
    }
}

/* ── 메쉬 로드 ─────────────────────────────────────────────── */
static int load_mesh(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 0; }
    char magic[4];
    if (fread(magic,1,4,f) != 4) { fclose(f); return 0; }
    /* 두 가지를 읽는다:
         M68G — 로봇 링크만 (관절 6개가 붙는다)
         M68S — 공장 전체 (로봇 + 바닥 + 설비, 조각마다 색이 따로) */
    scene_mode = !memcmp(magic,"M68S",4);
    if (!scene_mode && memcmp(magic,"M68G",4)) {
        fprintf(stderr, "형식 아님 (M68G/M68S 가 아니다)\n"); fclose(f); return 0; }
    uint16_t n;
    if (fread(&n,2,1,f) != 1) { fclose(f); return 0; }
    if (n > MAXPART) n = MAXPART;
    nlinks = n;
    for (int i = 0; i < n; i++) {
        uint16_t nv, nf;
        if (fread(&nv,2,1,f)!=1 || fread(&nf,2,1,f)!=1) break;
        int skipv = 0, skipf = 0;
        if (nv > MAXV) { skipv = nv - MAXV; nv = MAXV; }
        if (nf > MAXF) { skipf = nf - MAXF; nf = MAXF; }
        links[i].nv = nv; links[i].nf = nf;
        /* 공장 모드: 앞 7개가 로봇 링크다 (build_scene.py 가 그 순서로 쓴다).
           나머지는 바닥/설비 — 관절이 없다. */
        /* 공장 모드(M68S)는 앱이 이미 기구학을 풀어 세계좌표로 보낸 것이다.
           여기서 관절을 다시 돌리면 팔이 흩어진다 — 그대로 그린다.
           로봇만 담은 M68G 파일일 때만 관절을 붙인다. */
        part_joint[i] = scene_mode ? -1 : i;
        /* 팔레트 띠 = 조각 번호. 뷰어가 scene.json 의 색을 그대로 써서
           띠마다 진짜 색을 칠한다 (바닥은 회색, 로봇은 링크색). */
        part_band[i]  = i;
        for (int j = 0; j < nv; j++) {
            int16_t xyz[3];
            if (fread(xyz,2,3,f)!=3) break;
            /* 0.1mm 정수 → 16.16 고정소수점 */
            links[i].v[j].x = (int32_t)xyz[0] << FP;
            links[i].v[j].y = (int32_t)xyz[1] << FP;
            links[i].v[j].z = (int32_t)xyz[2] << FP;
        }
        if (skipv) fseek(f, (long)skipv*6, SEEK_CUR);
        for (int j = 0; j < nf; j++) {
            uint16_t abc[3];
            if (fread(abc,2,3,f)!=3) break;
            /* 잘려나간 정점을 가리키는 면은 버린다 (0으로 접어 면적 0) */
            links[i].f[j].a = abc[0] < nv ? abc[0] : 0;
            links[i].f[j].b = abc[1] < nv ? abc[1] : 0;
            links[i].f[j].c = abc[2] < nv ? abc[2] : 0;
        }
        if (skipf) fseek(f, (long)skipf*6, SEEK_CUR);
    }
    fclose(f);
    /* 전체 링크를 한 덩어리로 보고 중심을 원점으로, 크기를 화면에 맞춘다.
       (STL 좌표가 원점에서 멀리 떨어져 있어 그대로 두면 화면 밖으로 나간다) */
    {
        int32_t mn[3] = { 0x7fffffff, 0x7fffffff, 0x7fffffff };
        int32_t mx[3] = {-0x7fffffff,-0x7fffffff,-0x7fffffff};
        for (int i = 0; i < nlinks; i++)
            for (int j = 0; j < links[i].nv; j++) {
                V3 *v = &links[i].v[j];
                if (v->x<mn[0]) mn[0]=v->x;  if (v->x>mx[0]) mx[0]=v->x;
                if (v->y<mn[1]) mn[1]=v->y;  if (v->y>mx[1]) mx[1]=v->y;
                if (v->z<mn[2]) mn[2]=v->z;  if (v->z>mx[2]) mx[2]=v->z;
            }
        int32_t c[3];
        for (int k = 0; k < 3; k++) c[k] = (mn[k] + mx[k]) / 2;
        g_cx = c[0]; g_cy = c[1]; g_cz = c[2];
        for (int i = 0; i < nlinks; i++)
            for (int j = 0; j < links[i].nv; j++) {
                links[i].v[j].x -= c[0];
                links[i].v[j].y -= c[1];
                links[i].v[j].z -= c[2];
            }
        g_bmn.x = mn[0]-c[0]; g_bmn.y = mn[1]-c[1]; g_bmn.z = mn[2]-c[2];
        g_bmx.x = mx[0]-c[0]; g_bmx.y = mx[1]-c[1]; g_bmx.z = mx[2]-c[2];
        int32_t span = 0;
        for (int k = 0; k < 3; k++) { int32_t d = mx[k]-mn[k]; if (d > span) span = d; }
        printf("모델 중심 (%d,%d,%d)mm · 최대폭 %dmm\n",
               (c[0]>>FP)/UNIT, (c[1]>>FP)/UNIT, (c[2]>>FP)/UNIT, (span>>FP)/UNIT);
        printf("  보정 후 링크0 첫 정점: (%d,%d,%d)mm\n",
               links[0].v[0].x>>FP, links[0].v[0].y>>FP, links[0].v[0].z>>FP);
        g_span = span >> FP;                 /* 카메라 거리 결정에 쓴다 */
        /* 화면에 맞추기.
              화면픽셀 = 좌표 × fov / 거리
           거리를 span 의 배수로 잡으면 fov 는 단위와 무관한 상수가 된다.
           모델 최대폭이 화면 세로(H)의 약 90%를 차지하게 한다. */
        printf("  최대폭 %dmm — 화면 맞춤은 매 프레임 시점에 맞춰 다시 잡는다\n",
               g_span / UNIT);
    }
    return 1;
}

/* ── 한 프레임 그리기 ──────────────────────────────────────── */
/* 이번 프레임에서 공통 작업(변환·투영·컬링)에 든 사이클.
   삼각형 목록이 모든 노드에 뿌려지므로 이건 노드마다 중복해서 든다. */
static long long g_common_cyc;

static int draw_frame(const int *ang, int *out_drawn, int *out_culled) {
    memset(fb, 0, sizeof fb);
    for (int i = 0; i < MAXNODE; i++) node_cyc[i] = 0;
    cur_node = -1;
    long long _c0 = cycles;
    for (int i = 0; i < W*H; i++) zb[i] = 0x7fffffff;
    int drawn = 0, culled = 0;
    int tri_seq = 0;              /* 기하 단계를 노드끼리 나누는 순번 */

    /* 카메라: 로봇을 살짝 위에서 본다 */
    /* 로봇 중심(대략 x=100,y=-100,z=100mm)을 원점으로 끌어와 화면에 담는다 */
    /* 카메라: 모델 크기에 맞춰 뒤로 물러난 뒤, 위에서 비스듬히 본다.
       순서가 중요하다 — 회전을 먼저 적용하고 그 다음 이동이다. */
    /* STL 정점은 '조립된 상태의 절대 좌표' 다.
       관절 j를 θ 만큼 돌리려면: 관절 원점으로 평행이동 → 회전 → 되돌리기.
       그리고 자식 링크는 부모의 회전을 모두 물려받는다. */
    Mat acc[8];
    acc[0] = mat_ident();
    for (int L = 1; L <= 6 && L < nlinks; L++) {
        int32_t jx = (JOINT_XYZ[L-1][0]<<FP) - g_cx;
        int32_t jy = (JOINT_XYZ[L-1][1]<<FP) - g_cy;
        int32_t jz = (JOINT_XYZ[L-1][2]<<FP) - g_cz;
        Mat jrot = (JOINT_AXIS[L-1]==0) ? mat_rotx(ang[L-1])
                : (JOINT_AXIS[L-1]==1) ? mat_roty(ang[L-1])
                                       : mat_rotz(ang[L-1]);
        Mat local = mat_mul(mat_trans(jx,jy,jz),
                    mat_mul(jrot, mat_trans(-jx,-jy,-jz)));
        acc[L] = mat_mul(acc[L-1], local);
    }

    /* 회전 먼저, 이동은 그 다음. 모델을 원점 기준으로 돌린 뒤
       "돌린 결과" 의 한가운데를 화면 중앙으로 끌어온다.
       회전 전 좌표로 중심을 맞추면 돌릴 때마다 모델이 화면 밖으로 밀린다. */
    Mat rot = mat_mul(mat_rotx(g_pit), mat_roty(g_yaw));

    /* 돌린 뒤 실제 정점들의 경계를 잰다.
       경계상자 8꼭짓점만 쓰면 팔처럼 대각선으로 마른 모양은
       상자 한구석에만 몰려서 화면 가장자리로 밀린다. */
    int32_t rmn[3] = { 0x7fffffff, 0x7fffffff, 0x7fffffff };
    int32_t rmx[3] = {-0x7fffffff,-0x7fffffff,-0x7fffffff};
    for (int i = 0; i < nlinks; i++)
        for (int j = 0; j < links[i].nv; j++) {
            int _ji = part_joint[i];
            Mat _mi = _ji >= 0 ? mat_mul(rot, acc[_ji < 8 ? _ji : 7]) : rot;
            if (part_xf_on[i])
                _mi = mat_mul(_mi, mat_mul(mat_trans(-g_cx,-g_cy,-g_cz),
                      mat_mul(part_xf[i], mat_trans(g_cx,g_cy,g_cz))));
            V3 r = xform(_mi, links[i].v[j]);
            if (r.x<rmn[0]) rmn[0]=r.x;  if (r.x>rmx[0]) rmx[0]=r.x;
            if (r.y<rmn[1]) rmn[1]=r.y;  if (r.y>rmx[1]) rmx[1]=r.y;
            if (r.z<rmn[2]) rmn[2]=r.z;  if (r.z>rmx[2]) rmx[2]=r.z;
        }
    int32_t rcx = (rmn[0]+rmx[0])/2, rcy = (rmn[1]+rmx[1])/2;
    int32_t rw  = rmx[0]-rmn[0],     rh  = rmx[1]-rmn[1];
    /* 가로 320, 세로 240. 가로가 넘치면 가로가, 아니면 세로가 기준이다.
       세로 기준 크기로 환산해서 둘 중 큰 쪽을 쓴다. */
    int32_t rdw = (int32_t)((int64_t)rw * H / W);
    int32_t rd  = rdw > rh ? rdw : rh;
    if (rd <= 0) rd = 1;

    /* 거리는 모델 크기의 1.6배로 고정한다 (원근감이 자연스러운 값).
       확대/축소는 배율(fov) 로만 준다 — 거리와 배율을 같이 키우면
       둘이 상쇄돼 아무 변화도 없다. */
    int32_t dist = (int32_t)((int64_t)rd * 16 / 10);
    g_fov = (int)((double)H * 0.85 * dist / rd * (g_zoom / 160.0));

    Mat view = mat_mul(mat_trans(-rcx, -rcy, dist), rot);

    for (int L = 0; L < nlinks; L++) {
        /* 관절이 붙는 조각(로봇 링크)만 acc 를 곱한다.
           바닥 플레이트·설비는 제자리에 가만히 있어야 한다. */
        int _jL = part_joint[L];
        Mat mv = (_jL >= 0) ? mat_mul(view, acc[_jL < 8 ? _jL : 7]) : view;
        /* 앱이 보내준 '지금 자세' 가 있으면 그걸 쓴다.
           메쉬는 그대로 두고 행렬만 갈아끼우므로 다시 굽지 않아도 된다.

           주의: 정점은 load_mesh 에서 중심(g_c*)을 빼서 원점으로 당겨놨다.
           앱이 준 행렬은 원래 좌표 기준이므로 그냥 곱하면 회전 중심이
           어긋나 팔이 흩어진다. 되돌렸다가(+c) 적용하고 다시 당긴다(-c). */
        if (part_xf_on[L])
            mv = mat_mul(mv, mat_mul(mat_trans(-g_cx,-g_cy,-g_cz),
                 mat_mul(part_xf[L], mat_trans(g_cx,g_cy,g_cz))));
        Mesh *m = &links[L];
        for (int t = 0; t < m->nf; t++) {
            /* 기하 단계(변환·투영·컬링)는 삼각형을 노드끼리 나눠 맡는다.
               모든 노드가 전부 하면 그게 94% 를 먹어서 32개를 써도
               거의 안 빨라진다 (실측). 실제 어레이도 이렇게 나눈다:
                 · 기하: 삼각형 t 를 (t % 노드수) 번 노드가 맡는다
                 · 래스터: 그 결과를 화면 띠 담당 노드에 넘겨 그린다
               넘기는 값은 화면좌표 6개 + 깊이 3개 + 색 = 몇 바이트뿐이라
               링으로 보내도 부담이 안 된다. */
            cur_node = g_nodes > 1 ? (tri_seq++ % g_nodes) : -1;
            V3 a = xform(mv, m->v[m->f[t].a]);
            if (getenv("DBG2") && L==0 && t<3)
                fprintf(stderr,"  L0 t%d view=(%d,%d,%d)mm\n",t,
                        a.x>>FP, a.y>>FP, a.z>>FP);
            V3 b = xform(mv, m->v[m->f[t].b]);
            V3 c = xform(mv, m->v[m->f[t].c]);
            if (a.z <= ONE || b.z <= ONE || c.z <= ONE) { culled++; g_cull_z++; continue; }
            /* 68000 실물에서는 백페이스 컬링으로 절반을 버리지만,
               이 STL 은 법선 방향이 일정치 않아 뒷면도 그린다.
               (사이클 계산에는 컬링 있는 경우도 같이 보고한다) */

            /* 원근 투영 — DIVS 두 번 */
            /* 원근 투영. a.x, a.z 는 둘 다 16.16 고정소수점이라
               나누면 소수부가 상쇄돼 정수비가 나온다. 여기에 배율을 곱한다.
               (68000 은 DIVS.W 로 같은 일을 한다 — 158 사이클) */
            /* 화면좌표를 16.16 고정소수점으로 남긴다 — 소수부를 버리면
               1픽셀보다 작은 삼각형이 전부 사라진다. */
            /* 화면좌표 = 중심 + x*fov/z, 결과를 16.16 으로 남긴다.
               z 의 소수부까지 살리려고 분자를 미리 FP 만큼 올린다. */
            /* x,z 는 16.16. (x<<FP)/z 로 나누면 결과가 16.16 이 된다.
               중간값이 64비트를 넘으므로 128비트로 받는다.
               (68000 에서는 32비트 나눗셈 루틴을 쓴다 — 사이클은 아래서 센다) */
            #define PROJX(v) ((int32_t)((W/2)<<FP) + (int32_t)((((__int128)(v).x * g_fov) << FP) / (v).z))
            #define PROJY(v) ((int32_t)((H/2)<<FP) - (int32_t)((((__int128)(v).y * g_fov) << FP) / (v).z))
            int32_t ax = PROJX(a), ay = PROJY(a);
            int32_t bx = PROJX(b), by = PROJY(b);
            int32_t cx = PROJX(c), cy = PROJY(c);
            if (getenv("DBG3") && drawn+culled < 3)
                fprintf(stderr, "  proj: (%d,%d) (%d,%d) (%d,%d)  z=%d\n",
                        ax,ay,bx,by,cx,cy, a.z>>FP);
            CYC(C_DIVS*6);

            /* 백페이스 컬링 — 외적 z 부호 */
            /* 16.16 좌표의 외적은 32.32 다. 시프트로 줄이면 작은 면이
               0 으로 뭉개져 통째로 버려진다 (실측 909 중 434 소멸).
               전체 정밀도로 두고 진짜 0 인 것만 버린다. */
            long long cross = (long long)(bx-ax)*(cy-ay) - (long long)(by-ay)*(cx-ax);
            CYC(C_MULS*2 + C_CMP);
            if (cross == 0) { culled++; g_cull_a++; continue; }   /* 면적 0 만 버림 */
            int back = (cross < 0);                   /* 뒷면은 어둡게 */

            /* 플랫 셰이딩: 면 크기로 대충 밝기 (팔레트 인덱스) */
            /* 플랫 셰이딩 — 면이 빛을 얼마나 정면으로 받는지로 밝기를 정한다.
               전에는 '삼각형 면적÷둘레' 를 썼는데 그건 면의 모양일 뿐
               빛 각도가 아니다. 그래서 큰 면은 밝고 작은 면은 어두워져
               화면 전체가 뿌옇게 됐다.

               면 법선을 시점공간에서 직접 구한다 (외적 한 번).
               빛은 왼쪽 위 앞에서 온다 — 68000 도 이 정도는 한다. */
            V3 e1 = { b.x-a.x, b.y-a.y, b.z-a.z };
            V3 e2 = { c.x-a.x, c.y-a.y, c.z-a.z };
            /* 외적. 값이 커서 넘치지 않게 미리 줄인다 (16.16 → 정수 mm) */
            long long nx = (long long)(e1.y>>FP)*(e2.z>>FP) - (long long)(e1.z>>FP)*(e2.y>>FP);
            long long ny = (long long)(e1.z>>FP)*(e2.x>>FP) - (long long)(e1.x>>FP)*(e2.z>>FP);
            long long nz = (long long)(e1.x>>FP)*(e2.y>>FP) - (long long)(e1.y>>FP)*(e2.x>>FP);
            CYC(C_MULS*6 + C_ADD*3);
            /* 빛 방향 (-1, 1, 2) 를 정규화 없이 내적 — 부호와 크기만 쓴다 */
            long long dot = -nx + ny + 2*nz;
            long long len = (nx<0?-nx:nx) + (ny<0?-ny:ny) + (nz<0?-nz:nz) + 1;
            /* -1..1 을 0..7 로. 나눗셈 한 번 (68000 DIVS) */
            int shade = (int)((dot * 4) / len) + 4;
            CYC(C_DIVS);
            if (shade > 7) shade = 7;
            if (shade < 0) shade = 0;
            /* 팔레트: 조각마다 8칸씩 쓴다 (명암 8단계).
               8칸 × 29조각 = 232, 인덱스 16부터라 256 안에 들어간다. */
            uint8_t col = (uint8_t)(16 + (part_band[L] % 29) * 8 + shade);
            if (getenv("DBG") && drawn < 5)
                fprintf(stderr, "  tri%d L%d: (%d,%d) (%d,%d) (%d,%d)  z=%d\n",
                        drawn, L, ax,ay, bx,by, cx,cy, a.z>>16);
            /* 기하 담당 노드 -> 띠 담당 노드로 넘기는 비용.
               좌표 10개쯤을 시리얼로 보낸다. 링 한 칸에 MOVE.W 몇 번. */
            if (g_nodes > 1) CYC(C_ADD * 10);
            tri_fill(ax,ay,a.z, bx,by,b.z, cx,cy,c.z, col);
            drawn++;
        }
    }
    cur_node = -1;
    /* 전체 - 노드들이 나눠 쓴 것 = 모두가 중복으로 하는 공통 작업 */
    long long used = cycles - _c0, per_node_sum = 0;
    for (int i = 0; i < g_nodes && i < MAXNODE; i++) per_node_sum += node_cyc[i];
    g_common_cyc = used - per_node_sum;
    if (g_common_cyc < 0) g_common_cyc = 0;
    *out_drawn = drawn; *out_culled = culled;
    return drawn;
}

/* 이 프레임을 68000 어레이가 실제로 그리는 데 드는 사이클.
   = 가장 오래 걸린 노드. 모든 노드가 끝나야 화면이 나오므로
     전체를 노드 수로 나눈 값이 아니다 (그건 일이 완벽히 고를 때 이야기).
   각 노드는 '공통 작업 + 자기 띠' 를 한다. */
static long long frame_cycles(void) {
    if (g_nodes <= 1) return g_common_cyc + node_cyc[0];
    long long worst = 0;
    for (int i = 0; i < g_nodes && i < MAXNODE; i++) {
        long long t = g_common_cyc + node_cyc[i];
        if (t > worst) worst = t;
    }
    return worst;
}

/* ── PGM 저장 (확인용) ─────────────────────────────────────── */
static void save_pgm(const char *path) {
    FILE *f = fopen(path, "wb");
    fprintf(f, "P5\n%d %d\n255\n", W, H);
    fwrite(fb, 1, W*H, f);
    fclose(f);
}

/* ── 터미널에 그리기 (68000 속도 체감용) ─────────────────────
 *   실제 68000 이 이 프레임을 그리는 데 걸리는 시간만큼 기다렸다 표시한다.
 *   --live 8 32  →  8MHz × 32개 속도로 재생
 */
static void draw_ascii(void) {
    static const char *SH = " .:-=+*#%@";
    printf("\x1b[H");                      /* 커서 홈 */
    for (int y = 0; y < H; y += 5) {
        for (int x = 0; x < W; x += 2) {
            int v = fb[y*W+x];
            putchar(v ? SH[(v*9)/255 + 1 > 9 ? 9 : (v*9)/255 + 1] : ' ');
        }
        putchar('\n');
    }
    fflush(stdout);
}


/* ── 대화형 서브 모드 (--serve) ────────────────────────────────
 *   PL-68000 은 매 프레임 키보드/마우스를 읽어 시점을 바꾸고 다시 그린다.
 *   여기서는 그 "입력 → 다시 그리기" 를 표준입출력으로 옮겼다.
 *   웹 브라우저가 마우스를 움직이면 웹서버가 아래 한 줄을 써 넣는다:
 *       yaw pitch zoom joint_phase\n
 *   그러면 320x240 팔레트 프레임 한 장과 실제 68000 사이클 수를 돌려준다.
 *       "F" + w(2) + h(2) + cycles(8) + 픽셀 76800바이트
 *   프로세스는 계속 살아 있으므로 메쉬를 다시 읽지 않는다 (드래그가 부드럽다).
 */
static int serve_loop(void) {
    setvbuf(stdout, NULL, _IOFBF, 1 << 20);
    fputs("READY\n", stderr); fflush(stderr);
    char line[256];
    while (fgets(line, sizeof line, stdin)) {
        /* 'M <조각번호> <행렬 12개>' = 그 조각의 자세를 바꾼다.
           그림은 안 그리고 값만 받아둔다 (여러 줄 보내고 마지막에 그린다). */
        if (line[0] == 'M') {
            int idx; double m[12];
            if (sscanf(line + 1, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                       &idx, &m[0],&m[1],&m[2],&m[3], &m[4],&m[5],&m[6],&m[7],
                       &m[8],&m[9],&m[10],&m[11]) == 13
                && idx >= 0 && idx < MAXPART) {
                /* 회전 3x3 은 16.16 고정소수점, 이동은 0.1mm 단위 */
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++)
                        part_xf[idx].m[r*4+c] = (int32_t)(m[r*4+c] * 65536.0);
                    part_xf[idx].m[r*4+3] = (int32_t)(m[r*4+3] * 65536.0);
                }
                part_xf_on[idx] = 1;
            }
            continue;
        }
        if (line[0] == 'N') {          /* 'N <노드수>' = 어레이 구성 */
            int n = atoi(line + 1);
            if (n >= 1 && n <= MAXNODE) g_nodes = n;
            continue;
        }
        if (line[0] == 'R') {          /* 'R' = 자세 초기화 */
            for (int i = 0; i < MAXPART; i++) part_xf_on[i] = 0;
            continue;
        }
        double yaw = 28, pit = -18, zoom = 160, ph = 0;
        if (sscanf(line, "%lf %lf %lf %lf", &yaw, &pit, &zoom, &ph) < 3) continue;
        g_yaw  = ((int)yaw % 360 + 360) % 360;
        g_pit  = (int)pit;  if (g_pit >  89) g_pit =  89;
                            if (g_pit < -89) g_pit = -89;
        g_zoom = (int)zoom; if (g_zoom <  40) g_zoom =  40;
                            if (g_zoom > 600) g_zoom = 600;
        int ang[6];
        for (int j = 0; j < 6; j++)
            ang[j] = (int)(40 * sin(ph + j * 0.9));
        int dr, cu;
        draw_frame(ang, &dr, &cu);
        /* 어레이가 실제로 걸리는 시간 = 가장 느린 노드.
           전체 사이클을 노드 수로 나눈 값이 아니다. */
        long long used = frame_cycles();
        if (getenv("DBGN")) {
            long long mn=1LL<<62, mx=0, sum=0;
            for (int i=0;i<g_nodes&&i<MAXNODE;i++){
                if(node_cyc[i]<mn)mn=node_cyc[i];
                if(node_cyc[i]>mx)mx=node_cyc[i];
                sum+=node_cyc[i];
            }
            fprintf(stderr,"노드 %d개 · 공통 %lld (%.0f%%) · 띠 최소 %lld 최대 %lld 평균 %lld\n",
                g_nodes, g_common_cyc, 100.0*g_common_cyc/(g_common_cyc+mx),
                mn, mx, sum/(g_nodes?g_nodes:1));
        }
        uint16_t w = W, h = H;
        fputc('F', stdout);
        fwrite(&w, 2, 1, stdout);
        fwrite(&h, 2, 1, stdout);
        fwrite(&used, 8, 1, stdout);
        fwrite(fb, 1, W * H, stdout);
        fflush(stdout);
    }
    return 0;
}

int main(int argc, char **argv) {
    const char *mesh = argc > 1 ? argv[1] : "robot_lo.bin";
    int frames = argc > 2 ? atoi(argv[2]) : 1;
    int want_serve = 0;
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--serve")) want_serve = 1;
    init_trig();
    if (want_serve) {
        /* 메쉬 로딩 로그가 stdout 을 더럽히지 않게 잠시 stderr 로 돌린다 */
        fflush(stdout);
        int so = dup(1); dup2(2, 1);
        int ok = load_mesh(mesh);
        fflush(stdout); dup2(so, 1); close(so);
        if (!ok) return 1;
        return serve_loop();
    }
    if (!load_mesh(mesh)) return 1;

    int tot_f = 0, tot_v = 0;
    for (int i = 0; i < nlinks; i++) { tot_f += links[i].nf; tot_v += links[i].nv; }
    printf("메쉬: 링크 %d · 정점 %d · 삼각형 %d\n", nlinks, tot_v, tot_f);
    printf("화면: %dx%d 팔레트 8bit · 고정소수점 16.16 · 정수 전용\n\n", W, H);

    /* ── 프레임 시퀀스 덤프 (HTML 재생용) ── */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dump")) continue;
        int nf = (i+1 < argc) ? atoi(argv[i+1]) : 60;
        FILE *out = fopen("frames.bin", "wb");
        fwrite("M68F", 1, 4, out);
        uint16_t w = W, h = H, cnt = nf;
        fwrite(&w,2,1,out); fwrite(&h,2,1,out); fwrite(&cnt,2,1,out);
        long long c0 = cycles;
        for (int fr = 0; fr < nf; fr++) {
            int ang[6];
            for (int j = 0; j < 6; j++)
                ang[j] = (int)(40 * sin((fr*2*M_PI/nf) + j*0.9));
            int dr, cu;
            draw_frame(ang, &dr, &cu);
            fwrite(fb, 1, W*H, out);
        }
        fclose(out);
        double per = (double)(cycles - c0) / nf;
        /* 사이클 정보도 같이 저장 */
        FILE *j = fopen("frames.json", "w");
        fprintf(j, "{\"w\":%d,\"h\":%d,\"frames\":%d,\"cycles\":%.0f}\n",
                W, H, nf, per);
        fclose(j);
        printf("frames.bin  %d프레임 · 프레임당 %.0f 사이클\n", nf, per);
        return 0;
    }

    /* ── 실시간 재생 모드 ── */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--live")) continue;
        double mhz = (i+1 < argc) ? atof(argv[i+1]) : 12.5;
        int    ncpu= (i+2 < argc) ? atoi(argv[i+2]) : 32;
        printf("\x1b[2J");                  /* 화면 지우기 */
        for (int fr = 0; ; fr++) {
            int ang[6];
            for (int j = 0; j < 6; j++)
                ang[j] = (int)(40 * sin((fr*0.06) + j*0.9));
            long long before = cycles;
            int dr, cu;
            draw_frame(ang, &dr, &cu);
            double need = (double)(cycles - before) / (mhz*1e6) / ncpu;
            draw_ascii();
            printf("\n  68000 %.1fMHz × %d개 · 프레임 %d · %.0fms/프레임 (%.1f fps)"
                   "  [Ctrl+C 종료]   \n", mhz, ncpu, fr,
                   need*1000, 1.0/need);
            fflush(stdout);
            struct timespec ts = { (time_t)need, (long)((need - (long)need)*1e9) };
            nanosleep(&ts, NULL);
        }
    }

    long long c0 = cycles;
    int drawn = 0, culled = 0;
    int tri_seq = 0;              /* 기하 단계를 노드끼리 나누는 순번 */
    for (int fr = 0; fr < frames; fr++) {
        int ang[6];
        for (int j = 0; j < 6; j++)
            ang[j] = (int)(32 * sin((fr*0.08) + j*0.7));   /* 관절 흔들기 */
        draw_frame(ang, &drawn, &culled);
    }
    long long used = cycles - c0;
    double per_frame = (double)used / frames;

    printf("=== 68000 사이클 (프레임당) ===\n");
    printf("  총 사이클      %.0f\n", per_frame);
    printf("  그린 삼각형    %d · 컬링 %d (%.0f%% 제거)\n",
           drawn, culled, 100.0*culled/(drawn+culled));
    printf("    - 화면 뒤(z<=0) %d · 면적0 %d\n", g_cull_z, g_cull_a);
    printf("\n=== 실제 68000 에서의 성능 ===\n");
    struct { const char *nm; double mhz; int n; } cfg[] = {
        {"68000 8MHz  × 1",   8.0,  1},
        {"68000 12.5MHz × 1", 12.5, 1},
        {"68000 8MHz  × 32",  8.0,  32},
        {"68000 12.5MHz × 32",12.5, 32},
        {"68000 16.67MHz × 32",16.67,32},
    };
    for (unsigned i = 0; i < sizeof cfg/sizeof cfg[0]; i++) {
        double t = per_frame / (cfg[i].mhz*1e6) / cfg[i].n;
        printf("  %-22s %8.3f초/프레임   %6.1f fps\n", cfg[i].nm, t, 1.0/t);
    }
    save_pgm("frame.pgm");
    printf("\nframe.pgm 저장 (마지막 프레임)\n");
    return 0;
}
