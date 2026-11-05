# 68000 세계와 앱의 완전 분리

**68000 쪽은 파이썬 없이 혼자 도는 C 실행파일 `dusk` 가 됐다.**
앱(`start_pro.sh`)과 프로세스가 아예 다르므로, 어느 쪽을 뜯어고쳐도
다른 쪽은 영향을 받지 않는다.

---

## 지금 구조

```
./start_pro.sh                    dusk/dusk
  파이썬 앱                         C 실행파일 65KB
  포트 7900 / 7870                  포트 7880
  68000 코드 0줄                    파이썬 0줄
       │                                │
       └────── HTTP 로만 대화 (선택적) ───┘
                로봇 자세 받아오기
```

두 프로그램은 서로를 모른다. `dusk` 는 앱이 꺼져 있어도
저장된 장면(`gfx/scene.bin`)으로 혼자 돈다.

### 확인된 사실

```
$ ldd dusk
  libm.so.6      수학 (sin/cos)
  libc.so.6      표준 C
                 ← 파이썬 링크 0개

$ curl localhost:7880/api/status
  {"ok":true,"engine":"dusk (C)","python":false,"nodes":32,"w":320,"h":240}
```

앱이 떠 있든 꺼져 있든 `dusk` 는 그대로 돈다 (실제로 확인함).

---

## 왜 다시 했나 — 1차 분리는 가짜였다

처음에는 `arm_studio_pro.py` 에서 68000 코드 314줄을 떼어
`m68k_server.py` 로 옮기고 환경변수로만 붙게 했다.

그런데 그건 **파일만 갈라놓은 것**이었다:

```
./start_m68k.sh
   └─ python arm_studio_pro.py   ← 앱이 뜨고
        └─ import m68k_server     ← 그 안에 68000 이 얹힌다
```

같은 프로세스라 앱이 죽으면 68000 도 죽었다. 명령 하나에 두 개가
켜지는 것도 이 때문이었다. 그래서 **독립 실행파일**로 다시 만들었다.

---

## dusk — 실행법

```bash
cd Industry_gyu_arm/dusk
./dusk                        # 포트 7880, 68000 32노드
```

```
  칩 68000 · 8.0MHz · 노드 32개 · MAC 100사이클
  장면: 조각 15 · 삼각형 360 · 화면 320x240

  DUSK — 68000 어레이 (파이썬 없음, 순수 C)
  화면:  http://localhost:7880/
  종료:  Ctrl+C
```

### 인자

```
--port 7880              포트
--nodes 32               노드 수 (1~256)
--chip 68000             칩 (68000/68000-12/68020/68030/z80/6502/486dx2)
--mhz 8.0                클럭
--scene gfx/scene.bin    장면 파일
--weights rachel.bin     RACHEL 가중치 (LLM 도 올릴 때)
--ask "문장"             한 문장만 처리하고 끝 (서버 안 뜸)
```

칩 사이클표가 `dusk.h` 에 있어서 **칩을 갈아끼울 수 있다.**
나중에 6502·Z80 으로 시험할 때 `--chip` 만 바꾸면 된다.

### 빌드

```bash
gcc -O2 -o dusk main.c dusk.c dusk_gfx.c dusk_http.c dusk_node.c dusk_rachel.c -lm
```

외부 라이브러리 없음. Makefile 도 필요 없다.

---

## 파일 구성

| 파일 | 줄 | 하는 일 | 상태 |
|---|---|---|---|
| `main.c` | 113 | 진입점, 인자 처리 | 완성 |
| `dusk_http.c` | 258 | 자체 HTTP 서버 (파이썬 대체) | 완성 |
| `dusk_gfx.c` | 135 | `pl68k.c` 를 라이브러리로 감쌈 | 완성 |
| `dusk.c` | 194 | 어레이 시뮬 (노드 배분·사이클) | 완성 |
| `dusk_node.c` | 245 | 정수 전용 커널 (float 없음) | 완성 |
| `dusk_rachel.c` | 484 | RACHEL 추론 | **정확도 미해결** |

### 웹 API (전부 C 가 처리)

```
GET /                     뷰어 화면
GET /api/frame?yaw=&pitch=&zoom=&nodes=    화면 한 장 (320x240 팔레트)
GET /api/scene            장면 정보
GET /api/bench            구성별 실측 (노드 1~64)
GET /api/status           {"engine":"dusk (C)","python":false}
```

---

## 그래픽 — 완성, 검증됨

기존 `pl68k --serve` 와 대조했다.

| | pl68k (기존) | dusk (신규) | |
|---|---|---|---|
| 사이클 1노드 | 10,119,138 | 10,119,138 | 일치 |
| 사이클 32노드 | 792,880 | 792,880 | 일치 |
| 그림 | — | — | **다른 픽셀 0개** |

12.5MHz × 32노드: **63ms = 15.8 fps**

### 잡은 버그 — 화면이 새까맣게

첫 빌드에서 칠해진 픽셀이 0개였다. 원인은 `init_trig()` 를 안 부른 것.
사인 테이블이 전부 0이라 모든 회전이 0이 되고, 삼각형이 한 점으로
뭉개졌다. `pl68k.c` 의 `main()` 은 이걸 먼저 부르는데 어댑터가 빠뜨렸다.

---

## RACHEL — 골격만, 정확도 미해결

**여기가 다음 사람이 이어받을 곳이다.**

### 되는 것

- 가중치 로딩: **텐서 101개 · 9.6MB** 정상 (RCHL 형식)
- 사이클 집계: 68000 32개 기준 **3.6분**
- 정수 전용: `float`/`double` 한 글자도 안 씀

### 안 되는 것 — 출력이 비어 있다

```
$ ./dusk --weights ../rachel_cpp/rachel.bin --nodes 32 --ask "..."
출력: (비어 있음)
  걸리는 시간: 3.6분
```

첫 토큰에서 바로 `<eos>` 가 나온다. 진단해 보면:

```
$ DUSK_DBG=1 ./dusk ... --ask "..."
  pos56: [<eos>]25462529  [}]22405674  [l]20763960
```

정답은 `{"blocks": [...]` 로 시작해야 하는데 `<eos>` 가 1등이고,
후보들이 문맥과 무관하다 (`}`, `l`). **어텐션이 문맥을 못 읽고 있다.**

기존 C++ 엔진(`rachel_cpp/rachel`)은 같은 입력에 정답을 낸다:
```
{"blocks": ["@goto:d2", "@grasp:ppap", "@goto:d10", "@release:d10"]}
```

### 잡은 버그 — 잔차 포화

층마다 활성값이 32768 로 포화됐다 (`DUSK_DBG2=1` 로 확인).
잔차 연결에서 그냥 더해서 층마다 커진 것.

```c
/* 전 */ g_x[i] = clamp(g_x[i] + g_o[i]);           // L0 에서 이미 32768
/* 후 */ g_x[i] = (g_x[i] >> 1) + (g_o[i] >> 1);    // 14,000~18,000 대역
```

포화는 사라졌지만 출력은 여전히 비어 있다. **다른 원인이 더 있다.**

### 다음 사람이 볼 곳

의심 순서:

1. **어텐션 점수 스케일** (`dusk_rachel.c` 의 `/* 점수 */` 블록)
   `s >>= 15` 로 q30→q15 하는데, `dusk_mac_row` 결과가 이미 q30 이
   아닐 수 있다. q 와 k 의 스케일이 각각 다르게 재양자화되기 때문.

2. **`dusk_softmax_q` 의 exp 근사** (`dusk_node.c`)
   2의 거듭제곱 룩업으로 만들었는데 검증을 안 했다.
   입력이 [-1,0] 범위를 벗어나면 전부 0이나 전부 32767 이 될 수 있다.

3. **`ra_addbias` 의 시프트**
   `xshift` 를 항상 0으로 넘기고 있다 (`ra_addbias(g_acc, Bq, 3*d, 0)`).
   bias 스케일이 활성값 스케일과 안 맞을 것이다.

4. **임베딩 초기 스케일** (`(e + p) << 7`)
   대충 잡은 값이다. 근거가 없다.

### 검증 방법

```bash
# 정답 (기존 C++ 엔진)
cd rachel_cpp && ./rachel rachel.bin --serve

# 200개 전체 검증
python verify.py

# dusk 쪽
cd dusk && ./dusk --weights ../rachel_cpp/rachel.bin --ask "..."
DUSK_DBG=1  ./dusk ...   # logits 상위 3개
DUSK_DBG2=1 ./dusk ...   # 층별 활성값 크기
```

**층별로 값을 찍어 C++ 과 대조하는 게 가장 빠르다.**
어느 층에서 갈라지는지 찾으면 원인이 좁혀진다.

---

## 앱 쪽은 그대로

`start_pro.sh` 는 **한 글자도 안 건드렸다.**

`arm_studio_pro.py` 에는 이 블록만 남아 있다:

```python
if os.environ.get('ARM_STUDIO_LAUNCHER') == 'm68k':
    try:
        import m68k_server
        m68k_server.attach(app, core, JOB, start_job, STREAMS)
    except Exception as _e:
        print(f'[M68K] 붙이기 실패 (앱은 그대로 동작): {_e}')
```

| | start_pro.sh | start_m68k.sh |
|---|---|---|
| 전체 라우트 | 20개 | 30개 |
| m68k 라우트 | **0개** | 10개 |

`m68k_server.py` 는 **과도기 코드**다. 앱 안에서 68000 을 보고 싶을 때만
쓰는 통로이고, `dusk` 가 완성되면 지워도 된다.

---

## 앞으로 68000 을 고칠 때

`dusk/` 폴더 안에서만 작업하면 된다.

```
Industry_gyu_arm/
  start_pro.sh          ← 절대 건드리지 않는다
  arm_studio_pro.py     ← 건드리지 않는다 (환경변수 블록 5줄 빼고)
  arm_studio_web.py     ← 건드리지 않는다
  m68k_server.py        ← 과도기. dusk 완성되면 삭제 가능
  ──────────────────────────────────────────
  dusk/                 ← 여기서만 작업
    main.c  dusk.c  dusk_gfx.c  dusk_http.c  dusk_node.c  dusk_rachel.c
    dusk.h              칩 사이클표 · API 선언
    gfx/                pl68k.c · viewer.html · scene.bin
    emulate.py          어레이 에뮬 (파이썬, 검증용)
    mac_68k.s           68000 어셈블리 MAC 커널
    DESIGN.md           칩 선정·회로·예산
```

---

## 참고 수치

| | 값 | 출처 |
|---|---|---|
| 그래픽 (32노드 12.5MHz) | 63ms · 15.8fps | dusk 실측 |
| RACHEL (32노드 8MHz) | 3.6분 | dusk 사이클 집계 |
| RACHEL (32노드 8MHz) | 8.61분 | `emulate.py --quick` |

두 값이 다르다. `emulate.py` 는 통신까지 세고 `dusk` 는 계산 위주라
그렇다. **어느 쪽이 맞는지 확인이 필요하다.**
