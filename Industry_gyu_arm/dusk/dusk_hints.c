/* ============================================================
 *  색 → 도킹 힌트 만들기 (nano_llm/hints.py 의 C 판)
 *
 *  왜 필요한가 — MAKING_RACHEL.md 3장:
 *    색→도킹 조회를 모델에게 시켰더니 62.7% 에서 안 올라갔다.
 *    세 가지 방법을 시도했고 전부 실패했다 (62.7 → 65.2 → 64.0).
 *    그걸 33줄짜리 전처리로 빼냈더니 99.6% 가 됐다.
 *
 *    "사전 찾기는 신경망이 잘하는 일이 아니다" — 계산으로 완벽히
 *    되는 일은 계산에 맡긴다. 이게 이 프로젝트의 AI 경계선 원칙이다.
 *
 *  장면(scene.bin)의 조각 색을 읽어 색↔도킹 표를 만들고,
 *  문장에 나온 순서대로 도킹 id 를 나열한다.
 * ============================================================ */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dusk.h"

extern const char *dusk_gfx_scene_json(void);

/* ── 색 이름 ─────────────────────────────────────────────
   실사용에서 "오렌지 색으로 옮기자" 가 통째로 무시된 적이 있다 —
   목록에 없는 말은 힌트에 안 잡히고, 그러면 모델도 못 찾는다.
   ★ 긴 것부터 찾아야 "빨간색" 이 "빨간" 보다 먼저 잡힌다. */
typedef struct { const char *form; const char *std; } ColorForm;
static const ColorForm FORMS[] = {
    {"노르스름","노랑"},
    {"오렌지","주황"}, {"퍼플","보라"}, {"푸른","파랑"}, {"붉은","빨강"},
    {"빨간","빨강"}, {"빨강","빨강"}, {"레드","빨강"},
    {"주황","주황"}, {"주홍","주황"},
    {"노란","노랑"}, {"노랑","노랑"}, {"옐로","노랑"},
    {"초록","초록"}, {"녹색","초록"}, {"그린","초록"}, {"연두","초록"},
    {"파란","파랑"}, {"파랑","파랑"}, {"블루","파랑"},
    {"보라","보라"}, {"자주","보라"},
};
#define NFORMS ((int)(sizeof FORMS / sizeof FORMS[0]))

static const char *PEN_WORDS[] = { "펜", "부품", "물체", "펜대" };
#define NPEN ((int)(sizeof PEN_WORDS / sizeof PEN_WORDS[0]))

/* ── RGB → 색 이름 ───────────────────────────────────────
   장면의 조각 색을 사람이 쓰는 색 이름으로 바꾼다.
   가장 가까운 기준색을 고른다 (단순 거리). */
typedef struct { const char *name; int r, g, b; } NamedColor;
static const NamedColor PALETTE[] = {
    {"빨강", 230,  25,  75},
    {"주황", 245, 130,  49},
    {"노랑", 255, 225,  25},
    {"초록",  60, 180,  75},
    {"파랑",  67,  99, 216},
    {"보라", 145,  30, 180},
};
#define NPAL ((int)(sizeof PALETTE / sizeof PALETTE[0]))

static const char *rgb_to_name(int r, int g, int b)
{
    long best = -1; const char *bn = NULL;
    for (int i = 0; i < NPAL; i++) {
        long dr = r - PALETTE[i].r, dg = g - PALETTE[i].g, db = b - PALETTE[i].b;
        long d = dr*dr + dg*dg + db*db;
        if (best < 0 || d < best) { best = d; bn = PALETTE[i].name; }
    }
    /* 너무 멀면 색으로 안 친다 (회색 플레이트 등) */
    return best > 20000 ? NULL : bn;
}

/* ── 장면에서 색↔도킹 표를 뽑는다 ────────────────────────
   조각 id 가 "..._docking_station-5#10" 이면 도킹 10번이다. */
#define MAXDOCK 32
typedef struct {
    char  dock[MAXDOCK][8];      /* "d10" */
    const char *color[MAXDOCK];  /* "빨강" */
    int   n;
    char  home[8];               /* 펜이 있는 도킹 */
    const char *pcol;            /* 펜 색 */
} SceneMap;

/* JSON 에서 다음 "key": 값 을 찾는 아주 단순한 도우미 */
static const char *find_field(const char *p, const char *key)
{
    char pat[64];
    snprintf(pat, sizeof pat, "\"%s\"", key);
    return strstr(p, pat);
}

static int scene_map(SceneMap *m)
{
    memset(m, 0, sizeof *m);
    const char *j = dusk_gfx_scene_json();
    if (!j) return 0;

    /* "id": "..." 마다 kind 와 rgb 를 읽는다 */
    const char *p = j;
    while ((p = find_field(p, "id")) != NULL) {
        p = strchr(p + 4, '"'); if (!p) break;   /* 값 시작 */
        p++;
        const char *ide = strchr(p, '"'); if (!ide) break;
        char id[256]; size_t idl = (size_t)(ide - p);
        if (idl >= sizeof id) idl = sizeof id - 1;
        memcpy(id, p, idl); id[idl] = 0;
        p = ide + 1;

        /* 같은 조각의 kind 와 rgb */
        const char *k = find_field(p, "kind");
        const char *c = find_field(p, "rgb");
        const char *nx = find_field(p, "id");
        if (!k || !c) break;
        if (nx && (k > nx || c > nx)) continue;   /* 다음 조각으로 넘어감 */

        char kind[32] = {0};
        const char *ks = strchr(k + 6, '"');
        if (ks) { ks++; const char *ke = strchr(ks, '"');
                  if (ke) { size_t n = (size_t)(ke-ks);
                            if (n >= sizeof kind) n = sizeof kind-1;
                            memcpy(kind, ks, n); kind[n]=0; } }

        int r=0, g=0, b=0;
        const char *cs = strchr(c + 5, '[');
        if (cs) sscanf(cs + 1, " %d , %d , %d", &r, &g, &b);

        const char *cn = rgb_to_name(r, g, b);

        /* 도킹: id 에 "docking_station" 과 "#번호" 가 있다 */
        /* 조각 종류 이름이 두 가지다:
             env_box  — 옛 장면(상자로 뭉갠 것)
             env      — 지금 장면(원본 모양을 깎은 것)
           둘 다 받아준다. 안 그러면 장면을 다시 구웠을 때
           "도킹 색을 못 찾았습니다" 로 채팅이 통째로 죽는다. */
        if ((!strcmp(kind, "env_box") || !strcmp(kind, "env"))
            && strstr(id, "docking_station")) {
            const char *h = strrchr(id, '#');
            if (h && cn && m->n < MAXDOCK) {
                snprintf(m->dock[m->n], sizeof m->dock[0], "d%s", h + 1);
                m->color[m->n] = cn;
                m->n++;
            }
        }
        /* 펜: id 가 "ppap" */
        else if (!strcmp(id, "ppap")) {
            m->pcol = cn;
        }
    }

    /* 펜이 어느 도킹에 있는지 — scene.bin 의 정점 좌표로 찾는다.
       조각마다 중심을 구해, 펜(ppap)에서 가장 가까운 도킹을 고른다.
       ★ 전에는 첫 도킹으로 가정했는데, 목적지가 그 도킹이면
         "제자리로 옮기는" 엉뚱한 블록이 나왔다 (실측: d2 → d2). */
    if (m->n > 0) {
        int dusk_pen_dock(char (*)[8], int);
        int idx = dusk_pen_dock(m->dock, m->n);
        snprintf(m->home, sizeof m->home, "%s",
                 m->dock[idx >= 0 ? idx : 0]);
    }
    return m->n > 0;
}

/* ── 펜이 어느 도킹에 있나 — scene.bin 좌표로 판단 ────────
   형식(pl68k.c 와 같다): "M68S" + 조각수(u16) +
     조각마다 [nv u16][nf u16][정점 nv×3×i16][면 nf×3×u16]
   조각 순서는 scene.json 의 parts 순서와 같다.

   정점 평균으로 중심을 구하고, ppap 에서 가장 가까운 도킹을 고른다. */
extern const char *dusk_scene_path(void);

int dusk_pen_dock(char dock[][8], int ndock)
{
    const char *path = dusk_scene_path();
    if (!path) return -1;
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    char magic[4];
    if (fread(magic,1,4,f)!=4) { fclose(f); return -1; }
    uint16_t n;
    if (fread(&n,2,1,f)!=1) { fclose(f); return -1; }

    /* 조각별 중심 */
    static double cx[64], cy[64], cz[64];
    int np = n > 64 ? 64 : n;
    for (int i = 0; i < n; i++) {
        uint16_t nv, nf;
        if (fread(&nv,2,1,f)!=1 || fread(&nf,2,1,f)!=1) break;
        double sx=0, sy=0, sz=0;
        for (int j = 0; j < nv; j++) {
            int16_t xyz[3];
            if (fread(xyz,2,3,f)!=3) { nv = (uint16_t)j; break; }
            sx += xyz[0]; sy += xyz[1]; sz += xyz[2];
        }
        if (i < np && nv) { cx[i]=sx/nv; cy[i]=sy/nv; cz[i]=sz/nv; }
        fseek(f, (long)nf*6, SEEK_CUR);
    }
    fclose(f);

    /* scene.json 에서 ppap 와 각 도킹이 몇 번째 조각인지 찾는다 */
    const char *j = dusk_gfx_scene_json();
    if (!j) return -1;
    int pen_idx = -1;
    static int dock_idx[MAXDOCK];
    for (int i = 0; i < ndock; i++) dock_idx[i] = -1;

    const char *p = j; int part = 0;
    while ((p = find_field(p, "id")) != NULL) {
        p = strchr(p + 4, '"'); if (!p) break;
        p++;
        const char *e = strchr(p, '"'); if (!e) break;
        char id[256]; size_t l = (size_t)(e-p);
        if (l >= sizeof id) l = sizeof id - 1;
        memcpy(id, p, l); id[l] = 0;
        p = e + 1;

        if (!strcmp(id, "ppap")) pen_idx = part;
        else {
            const char *h = strrchr(id, '#');
            if (h && strstr(id, "docking_station")) {
                char dn[16]; snprintf(dn, sizeof dn, "d%s", h+1);
                for (int i = 0; i < ndock; i++)
                    if (!strcmp(dock[i], dn)) { dock_idx[i] = part; break; }
            }
        }
        part++;
    }
    if (pen_idx < 0 || pen_idx >= np) return -1;

    int best = -1; double bd = -1;
    for (int i = 0; i < ndock; i++) {
        int di = dock_idx[i];
        if (di < 0 || di >= np) continue;
        double dx=cx[di]-cx[pen_idx], dy=cy[di]-cy[pen_idx], dz=cz[di]-cz[pen_idx];
        double d = dx*dx + dy*dy + dz*dz;
        if (bd < 0 || d < bd) { bd = d; best = i; }
    }
    return best;
}

/* ── 힌트 만들기 (hints.py 의 build 와 같은 규칙) ──────── */
/* 장면에 없는 색을 말했을 때 기억해 둔다 (바깥에서 물어본다) */
static char g_missing[8][16];
static int  g_missing_n = 0;

int dusk_missing_colors(char *out, size_t outsz)
{
    if (!g_missing_n) { if (out && outsz) out[0] = 0; return 0; }
    size_t k = 0;
    for (int i = 0; i < g_missing_n && k < outsz - 20; i++)
        k += (size_t)snprintf(out + k, outsz - k, "%s%s",
                              i ? ", " : "", g_missing[i]);
    return g_missing_n;
}

static int hint_build(const SceneMap *m, const char *text,
                      char *out, size_t outsz)
{
    g_missing_n = 0;          /* 말할 때마다 새로 센다 */
    struct { int pos; const char *dock; } found[64];
    int nf = 0;
    size_t tl = strlen(text);

    /* 색 표기를 찾는다. 긴 것부터 (FORMS 가 그 순서다) */
    for (size_t i = 0; i < tl && nf < 64; ) {
        int matched = 0;
        for (int f = 0; f < NFORMS; f++) {
            size_t fl = strlen(FORMS[f].form);
            if (i + fl > tl || memcmp(text + i, FORMS[f].form, fl)) continue;

            /* 뒤에 '펜' 류가 오면 펜 얘기다 */
            char tail[16] = {0};
            size_t tn = tl - (i+fl); if (tn > 15) tn = 15;
            memcpy(tail, text + i + fl, tn);
            int is_pen = 0;
            for (int w = 0; w < NPEN; w++)
                if (strstr(tail, PEN_WORDS[w])) { is_pen = 1; break; }

            if (is_pen) {
                /* 펜 색과 같으면 펜이 있는 자리를 가리킨다 */
                if (m->pcol && !strcmp(FORMS[f].std, m->pcol) && m->home[0])
                    { found[nf].pos=(int)i; found[nf].dock=m->home; nf++; }
            } else {
                int hit = 0;
                for (int d = 0; d < m->n; d++)
                    if (!strcmp(m->color[d], FORMS[f].std))
                        { found[nf].pos=(int)i; found[nf].dock=m->dock[d];
                          nf++; hit = 1; break; }
                /* ★ 장면에 없는 색을 말했다 — 기억해 뒀다가 알려준다.
                     전에는 조용히 건너뛰어서, 사용자는 왜 그 말이
                     무시됐는지 알 수가 없었다 (실측: "보라색 도킹" 을
                     말했는데 장면엔 빨강·주황·노랑뿐이었다). */
                if (!hit && g_missing_n < 8) {
                    int dup = 0;
                    for (int q = 0; q < g_missing_n; q++)
                        if (!strcmp(g_missing[q], FORMS[f].std)) { dup = 1; break; }
                    if (!dup)
                        snprintf(g_missing[g_missing_n++], 16, "%s", FORMS[f].std);
                }
            }
            i += fl; matched = 1; break;
        }
        if (!matched) i++;
    }

    /* "8번" 처럼 번호로 부르는 것도 잡는다 */
    for (size_t i = 0; i + 1 < tl && nf < 64; i++) {
        if (text[i] < '0' || text[i] > '9') continue;
        int num = 0; size_t j = i;
        while (j < tl && text[j] >= '0' && text[j] <= '9') num = num*10 + (text[j++]-'0');
        size_t k = j; while (k < tl && text[k]==' ') k++;
        if (k + 2 < tl && !memcmp(text + k, "번", 3)) {
            char want[16]; snprintf(want, sizeof want, "d%d", num);
            for (int d = 0; d < m->n; d++)
                if (!strcmp(m->dock[d], want))
                    { found[nf].pos=(int)i; found[nf].dock=m->dock[d]; nf++; break; }
        }
        i = j - 1;
    }

    /* 문장에 나온 순서대로, 연속 중복만 제거 */
    for (int a = 0; a < nf; a++)
        for (int b = a+1; b < nf; b++)
            if (found[b].pos < found[a].pos) {
                int tp = found[a].pos; const char *td = found[a].dock;
                found[a] = found[b];
                found[b].pos = tp; found[b].dock = td;
            }

    out[0] = 0;
    const char *last = NULL;
    for (int a = 0; a < nf; a++) {
        if (last && !strcmp(last, found[a].dock)) continue;
        if (out[0]) strncat(out, " ", outsz - strlen(out) - 1);
        strncat(out, found[a].dock, outsz - strlen(out) - 1);
        last = found[a].dock;
    }
    return out[0] != 0;
}

/* ── 밖에서 부르는 것: 문장 → RACHEL 프롬프트 ───────────
   반환 0 이면 장면 정보가 없어서 못 만든 것이다. */
int dusk_make_prompt(const char *text, char *out, size_t outsz)
{
    SceneMap m;
    if (!scene_map(&m)) return 0;

    /* 장면 줄: "빨강=d10 노랑=d2 | 펜=초록@d10" */
    char scene[512] = {0};
    for (int i = 0; i < m.n; i++) {
        char one[64];
        snprintf(one, sizeof one, "%s%s=%s",
                 i ? " " : "", m.color[i], m.dock[i]);
        strncat(scene, one, sizeof scene - strlen(scene) - 1);
    }
    if (m.pcol && m.home[0]) {
        char one[64];
        snprintf(one, sizeof one, " | 펜=%s@%s", m.pcol, m.home);
        strncat(scene, one, sizeof scene - strlen(scene) - 1);
    }

    char hint[256] = {0};
    hint_build(&m, text, hint, sizeof hint);

    snprintf(out, outsz, "%s\n힌트 %s\n%s\n>", scene, hint, text);
    return 1;
}

/* ── 도킹 id → 색 이름 ────────────────────────────────────
   블록 라벨에 "빨강 도킹" 처럼 적으려고 dusk_blocks.c 가 부른다.
   여기 이미 색↔도킹 표가 있으니 새로 만들지 않고 빌려 쓴다. */
int dusk_dock_color(const char *dock, char *out, size_t outsz)
{
    if (!dock || !out) return 0;
    SceneMap m;
    if (!scene_map(&m)) return 0;
    for (int i = 0; i < m.n; i++)
        if (!strcmp(m.dock[i], dock) && m.color[i]) {
            snprintf(out, outsz, "%s", m.color[i]);
            return 1;
        }
    return 0;
}

/* ── 조각 좌표 읽기 (블록 실행용) ─────────────────────────
   dusk_exec.c 가 "d10 이 어디냐" 를 물을 때 쓴다.

   scene.bin 정점은 int16 · 0.1mm 단위다 (mm 로 반올림하면
   삼각형이 뭉개져서 그렇게 정했다). 미터로 바꿔 돌려준다.

   도킹은 '윗면' 이 필요하다 (거기서 +10+37 해서 잡는 높이가 나온다)
   -> z 는 최대값, xy 는 정점 평균. 앱의 docking_top 과 같은 방식이다. */
static int part_geom(const char *want_id, double *ox, double *oy,
                     double *oz_top, double *oz_mid)
{
    const char *path = dusk_scene_path();
    const char *j    = dusk_gfx_scene_json();
    if (!path || !j) return 0;

    /* 몇 번째 조각인지 */
    int idx = -1, part = 0;
    const char *p = j;
    while ((p = find_field(p, "id")) != NULL) {
        p = strchr(p + 4, '"'); if (!p) break;
        p++;
        const char *e = strchr(p, '"'); if (!e) break;
        char id[256]; size_t l = (size_t)(e - p);
        if (l >= sizeof id) l = sizeof id - 1;
        memcpy(id, p, l); id[l] = 0;
        p = e + 1;
        if (!strcmp(id, want_id)) { idx = part; break; }
        part++;
    }
    if (idx < 0) return 0;

    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    char magic[4]; uint16_t n;
    if (fread(magic,1,4,f)!=4 || fread(&n,2,1,f)!=1) { fclose(f); return 0; }

    int found = 0;
    for (int i = 0; i < n; i++) {
        uint16_t nv, nf;
        if (fread(&nv,2,1,f)!=1 || fread(&nf,2,1,f)!=1) break;
        if (i == idx) {
            double sx=0, sy=0, sz=0, zmax=-1e18;
            int cnt = 0;
            for (int k = 0; k < nv; k++) {
                int16_t v[3];
                if (fread(v,2,3,f)!=3) break;
                sx += v[0]; sy += v[1]; sz += v[2]; cnt++;
                if (v[2] > zmax) zmax = v[2];
            }
            if (cnt) {
                /* 0.1mm -> m */
                *ox = sx / cnt * 0.0001;
                *oy = sy / cnt * 0.0001;
                *oz_top = zmax   * 0.0001;
                *oz_mid = sz/cnt * 0.0001;
                found = 1;
            }
            break;
        }
        fseek(f, (long)nv*6 + (long)nf*6, SEEK_CUR);
    }
    fclose(f);
    return found;
}

/* 도킹 소켓 위치 — xy 는 조각 중심, z 는 윗면 */
int dusk_dock_xyz(const char *dock, double out[3])
{
    SceneMap m;
    if (!scene_map(&m)) return 0;
    /* d10 -> 실제 조각 id 를 scene.json 에서 찾는다 */
    const char *j = dusk_gfx_scene_json();
    if (!j) return 0;
    const char *p = j;
    while ((p = find_field(p, "id")) != NULL) {
        p = strchr(p + 4, '"'); if (!p) break;
        p++;
        const char *e = strchr(p, '"'); if (!e) break;
        char id[256]; size_t l = (size_t)(e - p);
        if (l >= sizeof id) l = sizeof id - 1;
        memcpy(id, p, l); id[l] = 0;
        p = e + 1;
        const char *h = strrchr(id, '#');
        if (h && strstr(id, "docking_station")) {
            char dn[16]; snprintf(dn, sizeof dn, "d%s", h + 1);
            if (!strcmp(dn, dock)) {
                double mid;
                return part_geom(id, &out[0], &out[1], &out[2], &mid);
            }
        }
    }
    return 0;
}

/* 펜 위치 — xy 중심, z 는 바닥(밑면) */
int dusk_pen_xyz(double out[3])
{
    double top, mid;
    if (!part_geom("ppap", &out[0], &out[1], &top, &mid)) return 0;
    /* 잡는 높이는 도킹에 꽂혀 있으면 162mm 고정이다.
       펜이 어느 도킹에 있는지 찾아 그 규칙을 쓴다 (앱과 같다). */
    SceneMap m;
    if (scene_map(&m) && m.home[0]) {
        double d[3];
        if (dusk_dock_xyz(m.home, d)) {
            out[2] = d[2] + 0.010 + 0.037;    /* 윗면 +10 +37 = 162mm */
            return 1;
        }
    }
    out[2] = mid;
    return 1;
}
