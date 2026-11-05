/* ============================================================
 *  DUSK — 68000 어레이, 독립 실행파일
 *
 *    ./dusk                      기본 (포트 7880, 32노드)
 *    ./dusk --port 7881          포트 바꾸기
 *    ./dusk --nodes 64           노드 수
 *    ./dusk --chip 68020         칩 바꾸기
 *    ./dusk --scene gfx/scene.bin
 *
 *  파이썬이 없어도 혼자 뜬다. 앱(arm_studio_pro.py)이 꺼져 있어도
 *  저장된 장면으로 계속 돈다.
 * ============================================================ */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>
#include <unistd.h>
#include <limits.h>

#include "dusk.h"

extern int  dusk_serve(int port, const char *root);
extern int  dusk_gfx_init(const char *scene_path);
extern void dusk_set_nodes(int n);

int main(int argc, char **argv)
{
    int   port  = 7880;
    int   nodes = 32;
    const char *chip  = "68000";
    double mhz  = 8.0;
    const char *scene = NULL;
    const char *weights = NULL;      /* RACHEL 가중치 (없으면 그래픽만) */
    const char *ask = NULL;          /* 한 문장 물어보고 끝내기 */

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--port")  && i + 1 < argc) port  = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--nodes") && i + 1 < argc) nodes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--chip")  && i + 1 < argc) chip  = argv[++i];
        else if (!strcmp(argv[i], "--mhz")   && i + 1 < argc) mhz   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--scene") && i + 1 < argc) scene = argv[++i];
        else if (!strcmp(argv[i], "--weights") && i + 1 < argc) weights = argv[++i];
        else if (!strcmp(argv[i], "--ask")  && i + 1 < argc) ask = argv[++i];
        else if (!strcmp(argv[i], "--force-any") || !strcmp(argv[i], "--force")) {
            extern int g_force_any; g_force_any = 1;
        }
        else if (!strcmp(argv[i], "--help")) {
            printf("사용법: %s [--port 7880] [--nodes 32] [--chip 68000]\n"
                   "        [--mhz 8.0] [--scene gfx/scene.bin]\n"
                   "        [--weights rachel.bin] [--ask \"문장\"]\n\n"
                   "  --weights 는 안 줘도 된다 — rachel_cpp/rachel_v9.bin 을 알아서 찾는다.\n"
                   "  다른 가중치를 쓰고 싶을 때만 준다.\n"
                   "  --ask 를 주면 한 문장만 처리하고 끝낸다 (서버 안 뜸).\n"
                   "  --force-any 는 포트를 쥔 게 dusk 가 아니어도 끄고 뺏는다.\n\n"
                   "칩 목록: ", argv[0]);
            for (int k = 0; k < DUSK_NCHIPS; k++)
                printf("%s%s", k ? " " : "", DUSK_CHIPS[k].name);
            printf("\n");
            return 0;
        }
    }

    /* 실행파일이 있는 폴더를 기준으로 삼는다 —
       어디서 실행해도 gfx/ 를 찾을 수 있게. */
    char exedir[PATH_MAX];
    ssize_t n = readlink("/proc/self/exe", exedir, sizeof exedir - 1);
    if (n > 0) { exedir[n] = 0; dirname(exedir); }
    else       { snprintf(exedir, sizeof exedir, "."); }

    char scene_buf[PATH_MAX], root[PATH_MAX];
    if (!scene) {
        snprintf(scene_buf, sizeof scene_buf, "%s/gfx/scene.bin", exedir);
        scene = scene_buf;
    }
    snprintf(root, sizeof root, "%s/gfx", exedir);

    /* ── 가중치를 안 줬으면 알아서 찾는다 ──────────────────────
       그냥 ./dusk 로 켜면 RACHEL 이 안 붙어서 채팅이
       "가중치가 없습니다" 로 죽었다. 정본부터 차례로 본다.
       (rachel_v9.bin 이 정본, rachel.bin 은 옛 v8) */
    static char wbuf[PATH_MAX];
    if (!weights) {
        static const char *CAND[] = {
            "../rachel_cpp/rachel_v9.bin",
            "../rachel_cpp/rachel.bin",
            "rachel_v9.bin",
            "rachel.bin",
        };
        for (unsigned i = 0; i < sizeof CAND / sizeof CAND[0]; i++) {
            snprintf(wbuf, sizeof wbuf, "%s/%s", exedir, CAND[i]);
            if (access(wbuf, R_OK) == 0) { weights = wbuf; break; }
        }
        if (weights)
            printf("  RACHEL 가중치: %s (자동)\n", weights);
        else
            printf("  RACHEL 가중치를 못 찾았습니다 — 그래픽만 돕니다\n"
                   "  (채팅을 쓰려면: ./dusk --weights 경로/rachel_v9.bin)\n");
    }

    const DuskChip *c = dusk_find_chip(chip);
    if (!c) { fprintf(stderr, "모르는 칩: %s (--help 로 목록)\n", chip); return 1; }

    printf("  칩 %s · %.1fMHz · 노드 %d개 · MAC %d사이클\n",
           c->name, mhz, nodes, c->t_mac);

    /* ── 포트를 먼저 잡는다 (서버로 뜰 때만) ──────────────────
       장면·가중치를 읽는 데 몇 초 걸리는데, 그걸 다 읽고 나서
       포트가 막혀 있으면 헛일이다. 자리부터 확보한다.
       이 안에서 이전에 켠 dusk 를 정리한다. */
    if (!ask && !dusk_claim_port(port)) return 1;

    if (!dusk_gfx_init(scene)) {
        fprintf(stderr,
          "\n장면 파일이 없습니다: %s\n"
          "앱을 한 번 켜서 장면을 구우면 만들어집니다:\n"
          "  cd .. && ./start_pro.sh   그리고 뷰어에서 [배치 다시 굽기]\n", scene);
        return 1;
    }
    dusk_set_nodes(nodes);

    /* ── RACHEL (선택) ── */
    if (weights) {
        if (!rachel_load(weights)) return 1;
        rachel_init(nodes, chip, mhz, "tree");
        { extern int g_rachel_ready; g_rachel_ready = 1; }
    }

    /* 한 문장만 처리하고 끝 — 검증용 */
    if (ask) {
        if (!weights) {
            fprintf(stderr, "--ask 를 쓰려면 --weights 도 필요합니다\n");
            return 1;
        }
        char out[1024];
        long long cyc = 0;
        rachel_run(ask, out, sizeof out, &cyc);
        double sec = rachel_seconds();
        printf("\n입력: %s\n출력: %s\n", ask, out);
        printf("%s %d개 · %.0f 사이클 · %s\n", chip, nodes, (double)cyc,
               sec > 60 ? "" : "");
        if (sec > 3600)      printf("  걸리는 시간: %.1f시간\n", sec / 3600);
        else if (sec > 60)   printf("  걸리는 시간: %.1f분\n", sec / 60);
        else                 printf("  걸리는 시간: %.1f초\n", sec);
        return 0;
    }

    return dusk_serve(port, root);
}
