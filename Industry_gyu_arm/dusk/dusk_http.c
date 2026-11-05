/* ============================================================
 *  DUSK 독립 웹 서버 — 파이썬 없이 혼자 뜬다
 *
 *  왜 만들었나
 *    전에는 68000 화면이 파이썬 앱(arm_studio_pro.py) 안에 얹혀 있었다.
 *    파일만 갈라놨지 같은 프로세스라, 앱이 죽으면 68000 도 죽고
 *    68000 이 앱을 멈출 수도 있었다.
 *
 *    이제 dusk 는 자기 포트(7880)로 혼자 뜬다. 파이썬이 꺼져 있어도
 *    저장된 장면으로 돈다. 앱과는 HTTP 로만 대화한다 —
 *    로봇 자세를 받아올 때 7870/state 를 읽는 것뿐이고, 실패하면
 *    마지막 자세로 계속 그린다.
 *
 *  외부 라이브러리 없음. POSIX 소켓만 쓴다.
 * ============================================================ */
#define _GNU_SOURCE      /* strcasestr */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <signal.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/time.h>

#include "dusk.h"

#define PORT_DEFAULT 7880
#define REQ_MAX      8192

/* 렌더러 쪽에서 제공 (dusk_gfx.c) */
extern int  dusk_gfx_init(const char *scene_path);
extern int  dusk_gfx_frame(double yaw, double pitch, double zoom,
                           double phase, unsigned char *out, long long *cycles);
extern void dusk_gfx_set_pan(int px, int py);
extern int  dusk_gfx_width(void);
extern int  dusk_gfx_height(void);
extern const char *dusk_gfx_scene_json(void);

/* RACHEL 추론 (dusk_rachel.c) — 가중치가 로드돼 있을 때만 동작 */
extern int  rachel_run(const char *prompt, char *out, int outmax, long long *cycles);
extern double rachel_seconds(void);
int  g_rachel_ready = 0;      /* main.c 가 가중치를 읽으면 1 로 세운다 */

/* 색→도킹 힌트 (dusk_hints.c) */
extern int dusk_make_prompt(const char *text, char *out, size_t outsz);
extern int dusk_missing_colors(char *out, size_t outsz);

/* 블록 보관 (dusk_blocks.c) */
extern int  dusk_blocks_set_from_json(const char *raw);
extern int  dusk_blocks_json(char *out, size_t outsz);
extern int  dusk_blocks_array(char *out, size_t outsz);

/* 블록 실행 (dusk_exec.c) */
extern int    dusk_exec_run(void);
extern int    dusk_exec_frames(void);
extern const double *dusk_exec_frame(int i);
extern const char   *dusk_exec_error(void);
extern void dusk_blocks_clear(void);
extern int  dusk_blocks_count(void);

/* 어레이 시뮬 (dusk.c) */
extern void dusk_set_nodes(int n);
extern int  dusk_get_nodes(void);

static int g_blk_pipe = -1; /* 자식일 때: 블록을 부모에게 보낼 통로 */
int g_force_any = 0;        /* --force-any: dusk 가 아닌 것도 끄고 포트를 뺏는다 */

static volatile sig_atomic_t g_run  = 1;
static volatile sig_atomic_t g_hits = 0;   /* Ctrl+C 를 몇 번 눌렀나 */

/* ★ Ctrl+C 가 안 먹던 이유
     signal() 로 걸면 리눅스에서는 SA_RESTART 가 붙는다. 그러면
     accept() 로 손님을 기다리는 중에 Ctrl+C 가 와도 커널이
     accept() 를 "다시 시작" 해버린다. g_run 은 0 이 되지만
     루프가 그 값을 보러 돌아오질 못해서 영영 안 꺼졌다.
     -> sigaction 으로 SA_RESTART 를 빼고 건다. 그래야 accept 가
        EINTR 로 튀어나오고 while (g_run) 이 조건을 다시 본다.

     두 번 누르면 즉시 끈다 — 추론(1초)이 돌고 있어도 기다리지 않게. */
static void on_sig(int s)
{
    (void)s;
    g_run = 0;
    if (++g_hits >= 2) _exit(130);
}

static void install_sigs(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_sig;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;                  /* ★ SA_RESTART 를 일부러 안 준다 */
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);
}

/* ── 유틸 ────────────────────────────────────────────────── */

static void send_all(int fd, const void *buf, size_t n)
{
    const char *p = (const char *)buf;
    while (n > 0) {
        ssize_t k = write(fd, p, n);
        if (k <= 0) { if (errno == EINTR) continue; return; }
        p += k; n -= (size_t)k;
    }
}

static void send_hdr(int fd, int code, const char *ctype,
                     size_t len, const char *extra)
{
    char h[512];
    int n = snprintf(h, sizeof h,
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %zu\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-store\r\n"
        "%s"
        "Connection: close\r\n\r\n",
        code, code == 200 ? "OK" : "Not Found", ctype, len,
        extra ? extra : "");
    send_all(fd, h, (size_t)n);
}

static void send_text(int fd, int code, const char *ctype, const char *body)
{
    size_t n = strlen(body);
    send_hdr(fd, code, ctype, n, NULL);
    send_all(fd, body, n);
}

/* 정적 파일 (viewer.html 등) */
static void send_file(int fd, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) { send_text(fd, 404, "text/plain", "not found"); return; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = (char *)malloc((size_t)sz);
    if (!buf || fread(buf, 1, (size_t)sz, f) != (size_t)sz) {
        fclose(f); free(buf);
        send_text(fd, 404, "text/plain", "read fail"); return;
    }
    fclose(f);

    const char *ct = "application/octet-stream";
    const char *dot = strrchr(path, '.');
    if (dot) {
        if      (!strcmp(dot, ".html")) ct = "text/html; charset=utf-8";
        else if (!strcmp(dot, ".js"))   ct = "application/javascript";
        else if (!strcmp(dot, ".css"))  ct = "text/css";
        else if (!strcmp(dot, ".json")) ct = "application/json; charset=utf-8";
    }
    send_hdr(fd, 200, ct, (size_t)sz, NULL);
    send_all(fd, buf, (size_t)sz);
    free(buf);
}

/* 질의문자열에서 수 하나 꺼내기 (?yaw=48&pitch=-22 …) */
static double qnum(const char *q, const char *key, double dflt)
{
    if (!q) return dflt;
    char pat[64];
    snprintf(pat, sizeof pat, "%s=", key);
    const char *p = strstr(q, pat);
    if (!p) return dflt;
    p += strlen(pat);
    return atof(p);
}

/* ── 요청 처리 ───────────────────────────────────────────── */

static char g_root[512];      /* 정적 파일 폴더 (gfx/) */

static void handle(int fd, const char *method, const char *path,
                   const char *query, const char *body)
{
    (void)method;

    /* ── 주소 정규화 ────────────────────────────────────────
       viewer.html 은 파이썬 앱 시절 주소인 /api/m68k/* 를 부른다.
       dusk 는 /api/* 로 제공한다. 접두사만 벗겨 같은 곳으로 보낸다.
       ─ 이렇게 하면 viewer.html 을 고치지 않아도 되고, 같은 파일이
         파이썬 앱(7900)에서도 dusk(7880)에서도 그대로 돈다. */
    if (!strncmp(path, "/api/m68k/", 10)) {
        static char norm[512];
        snprintf(norm, sizeof norm, "/api/%s", path + 10);
        path = norm;
    }

    /* 화면 한 장 그리기 */
    if (!strcmp(path, "/api/frame")) {
        int W = dusk_gfx_width(), H = dusk_gfx_height();
        int nodes = (int)qnum(query, "nodes", dusk_get_nodes());
        if (nodes >= 1 && nodes <= 256) dusk_set_nodes(nodes);

        unsigned char *px = (unsigned char *)malloc((size_t)(W * H));
        long long cyc = 0;
        /* 우클릭 수평 이동 — 렌더러에 알려준다 */
        dusk_gfx_set_pan((int)qnum(query, "panx", 0),
                         (int)qnum(query, "pany", 0));
        if (!px || !dusk_gfx_frame(qnum(query, "yaw",   48),
                                   qnum(query, "pitch", -22),
                                   qnum(query, "zoom",  160),
                                   qnum(query, "phase", 0),
                                   px, &cyc)) {
            free(px);
            send_text(fd, 404, "text/plain", "render fail");
            return;
        }
        char extra[128];
        snprintf(extra, sizeof extra,
                 "X-Cycles: %lld\r\nX-W: %d\r\nX-H: %d\r\n", cyc, W, H);
        send_hdr(fd, 200, "application/octet-stream", (size_t)(W * H), extra);
        send_all(fd, px, (size_t)(W * H));
        free(px);
        return;
    }

    /* 장면 정보 (조각 이름·색) */
    if (!strcmp(path, "/api/scene")) {
        const char *j = dusk_gfx_scene_json();
        send_text(fd, 200, "application/json; charset=utf-8", j ? j : "{}");
        return;
    }

    /* 구성별 실측 — 노드 수를 바꿔가며 한 장씩 그려본다 */
    if (!strcmp(path, "/api/bench")) {
        static const int NS[] = { 1, 4, 8, 16, 32, 64 };
        int W = dusk_gfx_width(), H = dusk_gfx_height();
        unsigned char *px = (unsigned char *)malloc((size_t)(W * H));
        char body[1024];
        int n = snprintf(body, sizeof body, "{\"rows\":[");
        int save = dusk_get_nodes();
        for (unsigned i = 0; i < sizeof NS / sizeof NS[0]; i++) {
            dusk_set_nodes(NS[i]);
            long long cyc = 0;
            dusk_gfx_frame(48, -22, 160, 0, px, &cyc);
            n += snprintf(body + n, sizeof body - (size_t)n,
                          "%s{\"nodes\":%d,\"cycles\":%lld}",
                          i ? "," : "", NS[i], cyc);
        }
        snprintf(body + n, sizeof body - (size_t)n, "]}");
        dusk_set_nodes(save);
        free(px);
        send_text(fd, 200, "application/json", body);
        return;
    }

    /* 상태 — 파이썬 없이 도는지 확인용 */
    if (!strcmp(path, "/api/status")) {
        char body[256];
        snprintf(body, sizeof body,
                 "{\"ok\":true,\"engine\":\"dusk (C)\",\"python\":false,"
                 "\"nodes\":%d,\"w\":%d,\"h\":%d}",
                 dusk_get_nodes(), dusk_gfx_width(), dusk_gfx_height());
        send_text(fd, 200, "application/json", body);
        return;
    }

    /* ── 아직 dusk 에 없는 기능 ─────────────────────────────
       viewer.html 은 파이썬 앱 시절 기능도 부른다 (블록·채팅·시뮬).
       그건 앱(core)에 붙어 있던 것이라 dusk 에는 없다.
       404 로 두면 화면에 붉은 오류가 뜨므로, 빈 결과를 정직하게
       돌려준다 — 없는 것을 있는 척하지는 않는다. */
    if (!strcmp(path, "/api/blocks") || !strcmp(path, "/api/blocks_clear")) {
        if (!strcmp(path, "/api/blocks_clear")) dusk_blocks_clear();
        char jb[8192];
        dusk_blocks_json(jb, sizeof jb);
        send_text(fd, 200, "application/json; charset=utf-8", jb);
        return;
    }
    if (!strcmp(path, "/api/simstatus")) {
        send_text(fd, 200, "application/json",
                  "{\"running\":false,\"prog\":\"\",\"error\":null}");
        return;
    }
    /* 뷰어는 d.msg 를 읽는다 (error 가 아니다). 이유를 그대로 보여준다 —
       "실패" 라고만 뜨면 무엇이 문제인지 알 수 없다. */
    if (!strcmp(path, "/api/say")) {
        if (!g_rachel_ready) {
            send_text(fd, 200, "application/json",
                      "{\"ok\":false,\"msg\":\"RACHEL 가중치가 없습니다. "
                      "./dusk --weights ../rachel_cpp/rachel.bin 으로 켜십시오.\"}");
            return;
        }
        /* 본문에서 text 를 꺼낸다 (아주 단순한 JSON 파싱) */
        char text[1024] = {0};
        const char *t = body ? strstr(body, "\"text\"") : NULL;
        if (t) {
            t = strchr(t + 6, '"');
            if (t) {
                t++;
                int n = 0;
                while (*t && *t != '"' && n < (int)sizeof text - 1) {
                    if (*t == '\\' && t[1]) {
                        t++;
                        /* JSON 이스케이프를 실제 문자로 */
                        if      (*t == 'n') { text[n++] = '\n'; t++; continue; }
                        else if (*t == 't') { text[n++] = '\t'; t++; continue; }
                        else if (*t == 'r') { t++; continue; }
                    }
                    text[n++] = *t++;
                }
                text[n] = 0;
            }
        }
        if (!text[0]) {
            send_text(fd, 200, "application/json",
                      "{\"ok\":false,\"msg\":\"할 말을 적어주십시오.\"}");
            return;
        }
        /* ★ RACHEL 은 '장면 + 힌트 + 문장' 형식으로 학습됐다.
             문장만 넣으면 엉뚱한 글자가 나온다 (실측: "승승승...").

             학습 형식:
               보라=d2 빨강=d10 | 펜=초록@d2
               힌트 d2 d10
               보라색 펜을 빨간 도킹으로 옮겨
               >

             dusk 는 아직 장면 정보(어느 도킹이 무슨 색인지)를 갖고
             있지 않다 — 그건 앱(arm_studio_web.py)의 hints.py 가 하던
             일이다. 그래서 사용자가 줄바꿈으로 직접 넣으면 그대로 쓰고,
             한 줄만 보내면 그렇게 하라고 알려준다. */
        /* ★ RACHEL 은 '장면 + 힌트 + 문장' 형식으로 학습됐다.
             문장만 넣으면 엉뚱한 글자가 나온다 (실측: "승승승...").

             색→도킹 표는 계산으로 완벽히 되므로 신경망에 안 시킨다 —
             MAKING_RACHEL.md 3장의 원칙이다 (62.7% → 99.6%).
             dusk_hints.c 가 장면(scene.bin)의 조각 색을 읽어 만든다. */
        char prompt[2048];
        if (strchr(text, '\n')) {
            /* 사용자가 장면까지 직접 넣었으면 그대로 쓴다 */
            snprintf(prompt, sizeof prompt, "%s\n>", text);
        } else if (!dusk_make_prompt(text, prompt, sizeof prompt)) {
            send_text(fd, 200, "application/json; charset=utf-8",
                "{\"ok\":false,\"msg\":\"장면에서 도킹 색을 못 찾았습니다. "
                "앱(7900)에서 [배치 다시 굽기] 를 해주십시오.\"}");
            return;
        }

        char out[1024] = {0};
        long long cyc = 0;
        rachel_run(prompt, out, sizeof out, &cyc);
        double sec = rachel_seconds();

        /* 결과를 JSON 으로 (따옴표만 이스케이프) */
        char esc[2048]; int e = 0;
        for (int i = 0; out[i] && e < (int)sizeof esc - 2; i++) {
            if (out[i] == '"' || out[i] == '\\') esc[e++] = '\\';
            esc[e++] = out[i];
        }
        esc[e] = 0;
        /* ★ 만들고 버리지 않는다 — 여기에 담아야 화면에 뜨고
             [블록 시뮬레이션] 이 돌릴 것이 생긴다. */
        int nb = dusk_blocks_set_from_json(out);
        /* 자식에서 돌고 있으면 부모에게도 알려준다 (안 그러면 사라진다) */
        if (g_blk_pipe >= 0) {
            ssize_t wr = write(g_blk_pipe, out, strlen(out));
            (void)wr;
        }
        char jb[8192];
        dusk_blocks_array(jb, sizeof jb);

        /* 장면에 없는 색을 말했으면 알려준다 — 조용히 무시하면
           "왜 그 말이 씹혔지?" 가 된다 */
        char miss[128] = {0}, warn[256] = {0};
        if (dusk_missing_colors(miss, sizeof miss) > 0)
            snprintf(warn, sizeof warn,
                     " (배치에 없는 색은 건너뜀: %s)", miss);

        char body2[12288];
        snprintf(body2, sizeof body2,
                 "{\"ok\":true,\"msg\":\"블록 %d개 · 68000 %d개로 %.1f분 걸립니다%s\","
                 "\"raw\":\"%s\",\"added\":%d,\"blocks\":%s}",
                 nb, dusk_get_nodes(), sec/60.0, warn, esc, nb, jb);
        send_text(fd, 200, "application/json; charset=utf-8", body2);
        return;
    }
    /* ── 블록 시뮬레이션 — 이제 dusk 가 직접 한다 ──────────
       예전에는 "앱 기능입니다" 하고 거절했다. 그래서 블록을
       만들어도 팔이 안 움직였다. 계산부를 C 로 옮겨 붙였다. */
    if (!strcmp(path, "/api/sim")) {
        if (!dusk_exec_run()) {
            const char *e = dusk_exec_error();
            char b[512];
            snprintf(b, sizeof b, "{\"ok\":false,\"msg\":\"%s\"}",
                     e ? e : "실행 실패");
            send_text(fd, 200, "application/json; charset=utf-8", b);
            return;
        }
        char b[256];
        snprintf(b, sizeof b,
                 "{\"ok\":true,\"frames\":%d,\"msg\":\"%d 자세로 돌립니다\"}",
                 dusk_exec_frames(), dusk_exec_frames());
        send_text(fd, 200, "application/json; charset=utf-8", b);
        return;
    }

    /* 재생용: n 번째 자세의 관절각 */
    if (!strcmp(path, "/api/simframe")) {
        int i = (int)qnum(query, "i", 0);
        const double *q = dusk_exec_frame(i);
        if (!q) { send_text(fd, 200, "application/json", "{\"ok\":false}"); return; }
        char b[256];
        snprintf(b, sizeof b,
                 "{\"ok\":true,\"n\":%d,\"q\":[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]}",
                 dusk_exec_frames(), q[0],q[1],q[2],q[3],q[4],q[5]);
        send_text(fd, 200, "application/json", b);
        return;
    }
    if (!strcmp(path, "/api/rebuild")) {
        send_text(fd, 200, "application/json",
                  "{\"ok\":false,\"msg\":\"배치 굽기는 앱(7900)에서 하십시오. "
                  "dusk 는 구워진 scene.bin 을 읽어 그립니다.\"}");
        return;
    }

    /* 정적 파일 */
    char full[1024];
    const char *rel = (!strcmp(path, "/")) ? "/viewer.html" : path;
    if (strstr(rel, "..")) { send_text(fd, 404, "text/plain", "no"); return; }
    snprintf(full, sizeof full, "%s%s", g_root, rel);
    send_file(fd, full);
}

/* ── 이전에 켠 dusk 찾아 정리 ─────────────────────────────
   /proc 을 직접 읽는다. pkill 같은 외부 명령을 부르지 않는다
   (파이썬 없음과 같은 이유로, 셸 의존도 없애는 게 낫다).

   자기 이름이 'dusk' 인 프로세스만 대상으로 하고,
   자기 자신(getpid)은 건너뛴다. 남의 프로그램은 손대지 않는다. */
static int kill_stale_dusk(int port)
{
    (void)port;
    DIR *d = opendir("/proc");
    if (!d) return 0;

    pid_t me = getpid();
    int n = 0;
    pid_t killed[64]; int nk = 0;
    struct dirent *e;
    while ((e = readdir(d))) {
        /* 숫자로만 된 이름 = 프로세스 */
        char *end;
        long pid = strtol(e->d_name, &end, 10);
        if (*end || pid <= 0 || (pid_t)pid == me) continue;

        /* comm = 실행 파일 이름 */
        char path[64], comm[64] = {0};
        snprintf(path, sizeof path, "/proc/%ld/comm", pid);
        FILE *f = fopen(path, "r");
        if (!f) continue;
        if (!fgets(comm, sizeof comm, f)) { fclose(f); continue; }
        fclose(f);
        char *nl = strchr(comm, '\n'); if (nl) *nl = 0;

        if (strcmp(comm, "dusk")) continue;      /* dusk 아니면 건너뛴다 */

        /* ★ 내 자식(요청 처리 중인 fork)은 절대 건드리지 않는다.
             say 요청을 자식에서 처리하는데, 이름이 같아서 정리 대상이
             됐다 — 첫 요청 뒤 서버가 죽던 원인이다. */
        {
            char pp[64]; FILE *pf; long ppid = 0;
            snprintf(pp, sizeof pp, "/proc/%ld/stat", pid);
            pf = fopen(pp, "r");
            if (pf) {
                char buf[1024];
                if (fgets(buf, sizeof buf, pf)) {
                    char *rp = strrchr(buf, ')');
                    if (rp) { char *tk = strtok(rp + 2, " ");   /* state */
                              tk = strtok(NULL, " ");            /* ppid  */
                              if (tk) ppid = atol(tk); }
                }
                fclose(pf);
            }
            if (ppid == (long)me) continue;      /* 내 자식이다 */
        }

        /* ★ 자기보다 나중에 뜬 것은 건드리지 않는다.
             pkill 직후 새로 띄우면 이전 것이 아직 종료 중인데, 그때
             새 프로세스가 '자기 형제'를 죽이며 꼬였다 (실측: 띄우자마자
             "종료합니다" 가 찍혔다). /proc/<pid>/stat 의 22번째 값이
             시작 시각(jiffies)이다 — 자기보다 오래된 것만 정리한다. */
        {
            char sp[64]; unsigned long long st_other = 0, st_me = 0;
            snprintf(sp, sizeof sp, "/proc/%ld/stat", pid);
            FILE *sf = fopen(sp, "r");
            if (sf) {
                char buf[1024];
                if (fgets(buf, sizeof buf, sf)) {
                    char *rp = strrchr(buf, ')');
                    if (rp) { int fld = 2; char *tk = strtok(rp + 2, " ");
                              while (tk && fld < 22) { tk = strtok(NULL, " "); fld++; }
                              if (tk) st_other = strtoull(tk, NULL, 10); }
                }
                fclose(sf);
            }
            sf = fopen("/proc/self/stat", "r");
            if (sf) {
                char buf[1024];
                if (fgets(buf, sizeof buf, sf)) {
                    char *rp = strrchr(buf, ')');
                    if (rp) { int fld = 2; char *tk = strtok(rp + 2, " ");
                              while (tk && fld < 22) { tk = strtok(NULL, " "); fld++; }
                              if (tk) st_me = strtoull(tk, NULL, 10); }
                }
                fclose(sf);
            }
            if (st_other && st_me && st_other >= st_me) continue;  /* 나보다 새것 */
        }

        /* ★ 먼저 곱게 부탁하고(SIGTERM), 안 들으면 강제로 끈다(SIGKILL).
             전에는 SIGTERM 만 보냈다. 추론 중이거나 신호를 못 받는
             상태면 안 죽고 포트를 계속 붙들어서, 결국 "포트를 못 씁니다"
             가 떴다 (실측). 이제는 확실히 정리한다. */
        if (kill((pid_t)pid, SIGTERM) != 0) continue;
        n++;
        if (nk < (int)(sizeof killed / sizeof killed[0]))
            killed[nk++] = (pid_t)pid;
    }
    closedir(d);

    /* 곱게 죽을 시간을 준다 (최대 1.5초) */
    for (int round = 0; round < 30 && nk; round++) {
        int alive = 0;
        for (int i = 0; i < nk; i++)
            if (killed[i] > 0 && kill(killed[i], 0) == 0) alive++;
        if (!alive) break;
        usleep(50000);
    }
    /* 그래도 버티면 강제 종료 */
    for (int i = 0; i < nk; i++) {
        if (killed[i] > 0 && kill(killed[i], 0) == 0) {
            kill(killed[i], SIGKILL);
            fprintf(stderr, "  이전 dusk(pid %d)가 안 꺼져서 강제 종료했습니다\n",
                    (int)killed[i]);
        }
    }
    if (nk) usleep(200000);       /* 커널이 포트를 놓을 틈 */
    return n;
}

/* ── 서버 ────────────────────────────────────────────────── */

static int g_srv = -1;      /* 미리 잡아둔 듣기 소켓 */

/* 무거운 요청을 맡긴 자식들 — 끌 때 이것만 골라서 정리한다 */
#define MAXKID 64
static pid_t g_kids[MAXKID];
static int   g_nkids = 0;

static void remember_kid(pid_t p)
{
    /* 이미 끝난 자리를 재활용한다 */
    for (int i = 0; i < g_nkids; i++)
        if (g_kids[i] <= 0) { g_kids[i] = p; return; }
    if (g_nkids < MAXKID) g_kids[g_nkids++] = p;
}

static void reap_kids(void)
{
    pid_t p;
    while ((p = waitpid(-1, NULL, WNOHANG)) > 0)
        for (int i = 0; i < g_nkids; i++)
            if (g_kids[i] == p) { g_kids[i] = 0; break; }
}

/* ── 포트를 쥔 놈을 찾아서 끈다 (이름과 무관) ────────────
   --force 일 때만 쓴다.

   방법: /proc/net/tcp 에서 그 포트의 LISTEN 소켓 inode 를 찾고,
   /proc/<pid>/fd/* 가 그 inode 를 가리키는 프로세스를 찾는다.
   ss 나 lsof 같은 외부 명령을 안 쓴다 (dusk 는 혼자 도는 게 원칙).

   ★ 자기 자신과 자기 자식은 절대 안 죽인다. */
static int kill_port_owner(int port, int me_only_dusk)
{
    char want[64];
    unsigned long inode = 0;

    for (int pass = 0; pass < 2 && !inode; pass++) {
        FILE *f = fopen(pass ? "/proc/net/tcp6" : "/proc/net/tcp", "r");
        if (!f) continue;
        char line[512];
        if (!fgets(line, sizeof line, f)) { fclose(f); continue; }  /* 헤더 */
        while (fgets(line, sizeof line, f)) {
            unsigned lport = 0, st = 0; unsigned long ino = 0;
            char la[128], ra[128];
            /* sl local rem st ... inode */
            if (sscanf(line, "%*d: %127[0-9A-Fa-f:] %127[0-9A-Fa-f:] %X %*X:%*X %*X:%*X %*X %*d %*d %lu",
                       la, ra, &st, &ino) < 4) continue;
            char *c = strrchr(la, ':');
            if (!c) continue;
            lport = (unsigned)strtoul(c + 1, NULL, 16);
            if (lport == (unsigned)port && st == 0x0A) {    /* 0A = LISTEN */
                inode = ino;
                break;
            }
        }
        fclose(f);
    }
    if (!inode) return 0;
    snprintf(want, sizeof want, "socket:[%lu]", inode);

    pid_t me = getpid();
    int n = 0;
    DIR *d = opendir("/proc");
    if (!d) return 0;
    struct dirent *e;
    while ((e = readdir(d))) {
        char *end;
        long pid = strtol(e->d_name, &end, 10);
        if (*end || pid <= 0 || (pid_t)pid == me) continue;

        /* 부모가 나면 내 자식이다 — 안 죽인다 */
        {
            char pp[64]; long ppid = 0;
            snprintf(pp, sizeof pp, "/proc/%ld/stat", pid);
            FILE *pf = fopen(pp, "r");
            if (pf) {
                char buf[1024];
                if (fgets(buf, sizeof buf, pf)) {
                    char *rp = strrchr(buf, ')');
                    if (rp) { char *tk = strtok(rp + 2, " ");
                              tk = strtok(NULL, " ");
                              if (tk) ppid = atol(tk); }
                }
                fclose(pf);
            }
            if (ppid == (long)me) continue;
        }

        char fdd[64];
        snprintf(fdd, sizeof fdd, "/proc/%ld/fd", pid);
        DIR *fd2 = opendir(fdd);
        if (!fd2) continue;                     /* 남의 프로세스 — 권한 없음 */
        struct dirent *fe;
        int hit = 0;
        while ((fe = readdir(fd2))) {
            char lp[160], tgt[128];
            snprintf(lp, sizeof lp, "/proc/%ld/fd/%s", pid, fe->d_name);
            ssize_t k = readlink(lp, tgt, sizeof tgt - 1);
            if (k <= 0) continue;
            tgt[k] = 0;
            if (!strcmp(tgt, want)) { hit = 1; break; }
        }
        closedir(fd2);
        if (!hit) continue;

        /* 이름 확인 — dusk 만 끌지, 아무거나 끌지 */
        char cp[64], comm[64] = {0};
        snprintf(cp, sizeof cp, "/proc/%ld/comm", pid);
        FILE *cf = fopen(cp, "r");
        if (cf) { if (fgets(comm, sizeof comm, cf)) {
                      char *nl = strchr(comm, '\n'); if (nl) *nl = 0; }
                  fclose(cf); }
        if (me_only_dusk && strcmp(comm, "dusk")) {
            fprintf(stderr,
                "  포트 %d 는 '%s'(pid %ld) 가 쓰고 있습니다 — dusk 가 아니라 두었습니다.\n"
                "  그래도 끄려면:  ./dusk --force-any\n", port, comm, pid);
            continue;
        }

        fprintf(stderr, "  포트 %d 를 쥔 %s(pid %ld) 를 끕니다\n", port, comm, pid);
        kill((pid_t)pid, SIGTERM);
        for (int r = 0; r < 30; r++) {
            if (kill((pid_t)pid, 0) != 0) break;
            usleep(50000);
        }
        if (kill((pid_t)pid, 0) == 0) {
            kill((pid_t)pid, SIGKILL);
            fprintf(stderr, "  안 꺼져서 강제 종료했습니다\n");
        }
        n++;
    }
    closedir(d);
    if (n) usleep(200000);
    return n;
}

/* ── 포트 자리 먼저 잡기 ──────────────────────────────────
   장면·가중치를 읽기 전에 부른다. 이전에 켠 dusk 가 있으면
   정리하고 이어받는다. 실패하면 0 을 돌려준다. */
int dusk_claim_port(int port)
{
    install_sigs();

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return 0; }
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);

    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 이 PC 안에서만 */
    a.sin_port = htons((unsigned short)port);
    /* ── 이전에 켠 dusk 를 정리하고 이어받는다 ────────────────
       켜기만 하면 알아서 새로 뜨게 한다 (사용자 요구).

       ★ 지키는 것 두 가지:
         1) 자기 자신은 죽이지 않는다 (getpid 비교)
         2) dusk 가 아닌 프로그램이 쓰고 있으면 건드리지 않는다
            — 남의 프로그램을 함부로 끄지 않는다. 알려주고 멈춘다. */
    if (bind(srv, (struct sockaddr *)&a, sizeof a) < 0) {
        /* ① 이름이 dusk 인 옛 프로세스 정리 (SIGTERM -> 안 되면 SIGKILL) */
        int killed = kill_stale_dusk(port);
        if (killed > 0) {
            printf("  이전에 켠 dusk %d개를 정리했습니다\n", killed);
            for (int i = 0; i < 20; i++) {
                if (bind(srv, (struct sockaddr *)&a, sizeof a) == 0) goto bound;
                usleep(100000);
            }
        }

        /* ② 그래도 안 되면 포트를 쥔 놈을 직접 찾아서 끈다.
             ①은 '이름이 dusk 인 것' 을 찾지만, setsid 로 띄웠거나
             이름이 다르면 못 찾는다 (실측: 그래서 못 켰다).
             여기서는 포트 소유자를 소켓 inode 로 직접 찾는다. */
        if (kill_port_owner(port, !g_force_any) > 0) {
            for (int i = 0; i < 30; i++) {
                if (bind(srv, (struct sockaddr *)&a, sizeof a) == 0) goto bound;
                usleep(100000);
            }
        }

        fprintf(stderr,
                "포트 %d 를 못 씁니다.\n"
                "  확인:  ss -tlnp | grep %d\n"
                "  아무 프로그램이나 끄고 뺏으려면:  ./dusk --force-any\n"
                "  다른 포트로:  ./dusk --port %d\n",
                port, port, port + 1);
        close(srv); return 0;
    }
bound:
    listen(srv, 16);
    g_srv = srv;
    return 1;
}

/* ── 서비스 시작 (자리는 이미 잡혀 있다) ───────────────── */
int dusk_serve(int port, const char *root)
{
    snprintf(g_root, sizeof g_root, "%s", root);
    if (g_srv < 0) {                      /* 혹시 안 잡았으면 지금 잡는다 */
        if (!dusk_claim_port(port)) return 1;
    }
    int srv = g_srv;
    int one = 1;                          /* TCP_NODELAY 용 */

    printf("\n  DUSK — 68000 어레이 (파이썬 없음, 순수 C)\n");
    printf("  화면:  http://localhost:%d/\n", port);
    printf("  종료:  Ctrl+C\n\n");
    fflush(stdout);

    char req[REQ_MAX];
    while (g_run) {
        int fd = accept(srv, NULL, NULL);
        /* EINTR 은 Ctrl+C 로 깬 것이다. g_run 이 0 이면 나간다. */
        if (fd < 0) { if (errno == EINTR && g_run) continue; break; }
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);

        /* ★ 타임아웃을 건다 — 없으면 브라우저가 연결만 열고 안 보낼 때
             영영 기다리며 서버 전체가 멈춘다 (실측: LISTEN 큐에 10건이
             쌓이고 모든 요청이 응답 없음). */
        struct timeval tv; tv.tv_sec = 5; tv.tv_usec = 0;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);

        ssize_t n = read(fd, req, sizeof req - 1);
        if (n > 0) {
            req[n] = 0;
            /* ★ POST 본문이 헤더와 나뉘어 오는 경우가 있다 (브라우저가
                 실제로 그렇게 보낸다). Content-Length 만큼 더 읽는다. */
            char *hdr_end = strstr(req, "\r\n\r\n");
            if (hdr_end) {
                const char *cl = strcasestr(req, "content-length:");
                if (cl) {
                    long want = atol(cl + 15);
                    long have = n - (hdr_end + 4 - req);
                    while (have < want && n < (ssize_t)sizeof req - 1) {
                        ssize_t k = read(fd, req + n, sizeof req - 1 - (size_t)n);
                        if (k <= 0) break;
                        n += k; have += k; req[n] = 0;
                    }
                }
            }
            char method[16] = {0}, url[1024] = {0};
            if (sscanf(req, "%15s %1023s", method, url) == 2) {
                char *q = strchr(url, '?');
                if (q) { *q = 0; q++; }
                /* POST 본문은 빈 줄 다음부터다 */
                char *body = strstr(req, "\r\n\r\n");
                if (body) body += 4;
                else { body = strstr(req, "\n\n"); if (body) body += 2; }
                /* ★ 무거운 요청(RACHEL 추론은 1초)은 자식에서 처리한다.
                     안 그러면 그동안 다른 요청이 전부 밀린다.
                     fork 가 실패하면 그냥 여기서 처리한다. */
                int heavy = !strncmp(url, "/api/say", 8)
                         || !strncmp(url, "/api/m68k/say", 13)
                         || !strncmp(url, "/api/bench", 10)
                         || !strncmp(url, "/api/m68k/bench", 15);
                /* ★ 자식이 만든 블록을 부모에게 돌려줄 통로.
                     say 는 자식에서 도는데(추론 1초), 블록을 자식
                     메모리에만 담으면 자식이 끝날 때 같이 사라진다.
                     실제로 그랬다 — 화면엔 떴다가 새로고침하면
                     "블록이 없습니다" 로 돌아갔다. */
                int bpipe[2] = { -1, -1 };
                if (heavy && pipe(bpipe) != 0) { bpipe[0] = bpipe[1] = -1; }
                pid_t kid = heavy ? fork() : -1;
                if (kid == 0) {            /* 자식 */
                    if (bpipe[0] >= 0) close(bpipe[0]);
                    g_blk_pipe = bpipe[1];
                    /* 자식은 Ctrl+C 에 그냥 죽는다 — 부모가 먼저 꺼졌는데
                       추론하던 자식만 남아 포트를 붙들고 있으면
                       다음에 켤 때 "포트를 못 씁니다" 가 난다. */
                    signal(SIGINT,  SIG_DFL);
                    signal(SIGTERM, SIG_DFL);
                    close(srv);            /* 듣기 소켓은 부모 것이다 */
                    handle(fd, method, url, q, body);
                    close(fd);
                    if (g_blk_pipe >= 0) close(g_blk_pipe);
                    _exit(0);
                } else if (kid < 0) {      /* fork 안 됨 -> 직접 */
                    if (bpipe[0] >= 0) { close(bpipe[0]); close(bpipe[1]); }
                    handle(fd, method, url, q, body);
                } else {
                    remember_kid(kid);
                    /* 자식이 보내온 블록 원문을 받아 부모에도 담는다 */
                    if (bpipe[0] >= 0) {
                        close(bpipe[1]);
                        char rbuf[4096]; size_t rn = 0; ssize_t k;
                        while (rn < sizeof rbuf - 1 &&
                               (k = read(bpipe[0], rbuf + rn,
                                         sizeof rbuf - 1 - rn)) > 0)
                            rn += (size_t)k;
                        rbuf[rn] = 0;
                        close(bpipe[0]);
                        if (rn > 0) dusk_blocks_set_from_json(rbuf);
                    }
                }
            }
        }
        /* 좀비 정리 */
        reap_kids();
        close(fd);
    }
    close(srv);
    /* 추론하던 자식이 남아 있으면 같이 정리한다.
       ★ kill(0, ...) 을 쓰면 안 된다 — 그건 프로세스 그룹 전체라서
         터미널에서 그냥 ./dusk 로 켰을 때 사용자의 셸까지 같이
         죽는다. 내가 낳은 자식만 골라서 보낸다. */
    for (int i = 0; i < g_nkids; i++)
        if (g_kids[i] > 0) kill(g_kids[i], SIGTERM);
    for (int i = 0; i < 20; i++) {          /* 최대 1초 기다려준다 */
        if (waitpid(-1, NULL, WNOHANG) <= 0 && errno == ECHILD) break;
        usleep(50000);
    }
    printf("\n종료합니다.\n");
    return 0;
}
