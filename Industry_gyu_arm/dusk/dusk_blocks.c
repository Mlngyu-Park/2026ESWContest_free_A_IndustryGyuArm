/* ============================================================
 *  블록 보관 — RACHEL 이 만든 블록을 담아두고 화면에 보여준다
 *
 *  왜 필요했나
 *    dusk 를 start_pro 에서 떼어낼 때 3D 화면과 RACHEL 은 옮겨왔는데
 *    그 둘을 잇는 부분을 안 옮겼다. 그래서 이런 일이 벌어졌다:
 *
 *      RACHEL 이 블록을 만든다  →  raw 로 돌려준다  →  아무데도 안 남는다
 *      화면은 "블록이 없습니다"  →  [블록 시뮬레이션] 을 눌러도 안 움직인다
 *
 *    만들고 버렸던 것이다. 여기서 받아 둔다.
 *
 *  라벨 형식은 앱(arm_studio_web.py 의 seq_labels)과 똑같이 맞춘다.
 *  같은 명령이면 앱에서든 dusk 에서든 같은 글자가 보여야 한다.
 *
 *      ➜ 0. 이동: 빨강 도킹      (@goto:d10)
 *      ◈ 1. 잡기                 (@grasp:ppap)
 *      ▼ 2. 하강 (그 자리 수직)  (@down)
 *      ◆ 3. 그리퍼 열기 ✋        (@open)
 *
 *  실수 없이 옮긴다 — 라벨을 새로 지어내지 않는다.
 * ============================================================ */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dusk.h"

/* 색 이름을 붙이려고 힌트 쪽 표를 빌린다 */
extern int dusk_dock_color(const char *dock, char *out, size_t outsz);

#define MAXBLK 64

typedef struct {
    char item[64];      /* 원문 그대로: "@goto:d10" */
    char label[160];    /* 화면에 보일 글자 */
} Block;

static Block g_blk[MAXBLK];
static int   g_nblk  = 0;   /* 화면 줄 수 (이동 줄 포함) */
static int   g_nitem = 0;   /* 실행되는 블록 수 (번호용) */
static int   g_nseg  = 0;   /* 이동 구간 번호 */
static int   g_had_pose = 0;

int dusk_blocks_count(void) { return g_nitem; }

void dusk_blocks_clear(void)
{
    g_nblk = g_nitem = g_nseg = g_had_pose = 0;
}

/* 실행용: n 번째 '진짜' 블록 (이동 줄은 건너뛴다) */
const char *dusk_block_item(int n)
{
    int k = 0;
    for (int i = 0; i < g_nblk; i++) {
        if (!g_blk[i].item[0]) continue;      /* 보여주기용 이동 줄 */
        if (k == n) return g_blk[i].item;
        k++;
    }
    return NULL;
}

/* ── 대상 id 를 사람이 읽는 이름으로 ──────────────────────
   "d10" -> "빨강 도킹". 색을 못 찾으면 id 를 그대로 쓴다 —
   모르는 것을 아는 척하지 않는다. */
static void nice_name(const char *oid, char *out, size_t outsz)
{
    /* ★ 앱(disp_name)과 같아야 한다.
         앱은 등록된 이름이 없으면 id 를 그대로 쓴다 — 실제 화면에
         "이동: ppap", "이동: docking_station-5#10" 으로 나온다.
         여기서만 "펜"/"빨강 도킹" 으로 지어내면 같은 명령인데
         앱과 dusk 의 글자가 달라진다. 그래서 id 를 쓴다.
         (dusk 는 도킹을 d10 처럼 짧게 부르므로 그대로 둔다) */
    snprintf(out, outsz, "%s", oid);
}

/* ── 블록 하나 추가 ───────────────────────────────────────
   라벨 규칙은 앱의 seq_labels 를 그대로 따른다.

   ★ 번호는 '실행되는 블록' 만 센다. 앱도 그렇게 센다 —
     사이에 끼는 "┃ 이동" 줄은 보여주기용이라 번호를 안 먹는다.
     (앱 실물: ➜0 ◈1 ┃0↓ ➜2 ◈3 — ┃ 다음이 2 로 이어진다) */
static int add_one(const char *item)
{
    if (g_nblk >= MAXBLK) return 0;

    /* 자세 블록(➜)이 앞서 한 번 나왔으면 그 사이에 이동 줄을 넣는다 */
    if (!strncmp(item, "@goto:", 6) && g_had_pose && g_nblk < MAXBLK) {
        Block *sep = &g_blk[g_nblk];
        sep->item[0] = 0;                 /* 실행 대상이 아니다 */
        snprintf(sep->label, sizeof sep->label,
                 "┃ %d↓ 이동 · 간편 · 자동 경로", g_nseg++);
        g_nblk++;
        if (g_nblk >= MAXBLK) return 0;
    }

    Block *b = &g_blk[g_nblk];
    snprintf(b->item, sizeof b->item, "%s", item);

    int i = g_nitem;
    if (!strncmp(item, "@goto:", 6)) {
        char nm[96]; nice_name(item + 6, nm, sizeof nm);
        snprintf(b->label, sizeof b->label, "➜ %d. 이동: %s", i, nm);
    } else if (!strncmp(item, "@grasp:", 7)) {
        snprintf(b->label, sizeof b->label, "◈ %d. 잡기", i);
    } else if (!strncmp(item, "@release:", 9)) {
        snprintf(b->label, sizeof b->label, "◈ %d. 놓기", i);
    } else if (!strcmp(item, "@open")) {
        snprintf(b->label, sizeof b->label, "◆ %d. 그리퍼 열기 ✋", i);
    } else if (!strcmp(item, "@close")) {
        snprintf(b->label, sizeof b->label, "◆ %d. 그리퍼 닫기 ✊", i);
    } else if (!strcmp(item, "@down")) {
        snprintf(b->label, sizeof b->label, "▼ %d. 하강 (그 자리 수직)", i);
    } else if (!strcmp(item, "@up")) {
        snprintf(b->label, sizeof b->label, "▲ %d. 상승 (그 자리 수직)", i);
    } else {
        snprintf(b->label, sizeof b->label, "■ %d. %s", i, item);
    }
    if (!strncmp(item, "@goto:", 6)) g_had_pose = 1;
    g_nblk++; g_nitem++;
    return 1;
}

/* ── RACHEL 출력에서 블록을 뽑아 담는다 ───────────────────
   받는 글자: {"blocks": ["@goto:d2", "@grasp:ppap", ...]}
   따옴표 안의 "@..." 만 순서대로 가져온다.

   ★ 이전 것을 지우고 새로 담는다 — 말할 때마다 쌓이면
     엉뚱한 옛 블록까지 같이 돈다. */
int dusk_blocks_set_from_json(const char *raw)
{
    dusk_blocks_clear();
    if (!raw) return 0;

    const char *p = raw;
    while ((p = strchr(p, '"')) != NULL) {
        p++;
        const char *e = strchr(p, '"');
        if (!e) break;
        size_t n = (size_t)(e - p);
        if (n > 0 && n < 64 && *p == '@') {
            char it[64];
            memcpy(it, p, n); it[n] = 0;
            add_one(it);
        }
        p = e + 1;
    }
    return g_nitem;
}

/* ── 화면에 줄 JSON ──────────────────────────────────────
   뷰어(viewer.html 의 drawBlocks)는 b.label 을 읽는다. */
/* 배열만 만든다: [{"label":"..."},...]
   감싸는 {"blocks": ...} 는 부르는 쪽이 붙인다 — 전에 문자열을
   글자수로 잘라 붙였더니 형식이 바뀌면 깨질 자리였다. */
int dusk_blocks_array(char *out, size_t outsz)
{
    size_t k = 0;
    k += (size_t)snprintf(out + k, outsz - k, "[");
    for (int i = 0; i < g_nblk && k < outsz - 64; i++) {
        /* 라벨에 "  나 \ 는 안 들어가지만, 그래도 막아둔다 */
        char esc[192]; size_t e = 0;
        for (const char *s = g_blk[i].label; *s && e < sizeof esc - 2; s++) {
            if (*s == '"' || *s == '\\') esc[e++] = '\\';
            esc[e++] = *s;
        }
        esc[e] = 0;
        k += (size_t)snprintf(out + k, outsz - k, "%s{\"label\":\"%s\"}",
                              i ? "," : "", esc);
    }
    k += (size_t)snprintf(out + k, outsz - k, "]");
    return (int)k;
}

/* {"blocks":[...]} 통째로 */
int dusk_blocks_json(char *out, size_t outsz)
{
    char arr[8192];
    dusk_blocks_array(arr, sizeof arr);
    return snprintf(out, outsz, "{\"blocks\":%s}", arr);
}
