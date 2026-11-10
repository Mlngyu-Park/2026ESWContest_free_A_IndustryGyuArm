# DUSK_BOOM — 직접 설계한 행렬곱 TPU (Zynq XC7Z010)

DUSK_BOOM 은 Zynq 7010 FPGA 보드에 **Verilog 로 직접 설계한 행렬곱 전용 TPU** 다.
GPU 없이, 적은 전력으로 [RACHEL](RACHEL.md)(더블 레이첼)을 돌리기 위해 만들었다.

회로(Verilog), 보드 리눅스 부팅, HDMI 출력 회로, TPU 를 부르는 보드 프로그램(C++), 시연 화면까지 전부 이 저장소에 있다.

> TPU 작동 원리와 비바도로 만드는 순서(타이밍을 맞춘 빌드 4번 포함)는 **[TPU.md](TPU.md)** 에 따로 정리했다.

---

## 1. 왜 TPU 인가

**저전력** — 같은 AI 추론을 할 때

| 장치 | AI 추론 때 전력 |
|---|---|
| Intel N305 CPU (LattePanda Mu) | 최대 21W |
| **Xilinx XC7Z010 (DUSK_BOOM)** | **최대 2.5W** (비바도 전력 분석: 기본 1.96W + TPU 가동분 0.55W) |

**안성맞춤 설계** — 언어 모델이 쓰는 시간은 대부분 **"가중치 행렬 × 입력 벡터"** 계산이다.
행렬의 각 행을 곱하는 일은 **서로를 기다릴 필요가 없다.** 그래서 곱셈기를 여러 개 깔아 두고 동시에 돌리면 빨라진다.
CPU 는 이 곱셈을 몇 개씩 차례로 하지만, TPU 는 곱셈기 64개가 한 클럭에 같이 곱한다.

---

## 2. 한눈에 보기

| 항목 | 값 |
|---|---|
| 보드 | ALINX AC7010C 코어 모듈을 얹은 DUSK_BOOM 보드 (ESP32-S3 · HDMI · USB) |
| 칩 | Xilinx Zynq XC7Z010 (FPGA + ARM Cortex-A9 2코어) |
| 메모리 | DDR3 512MB (그중 32MB 를 TPU 전용으로 떼어 둠) |
| TPU | 엔진 2개 × (가중치 8개 × 입력 벡터 4개) = **곱셈기 64개** (8×8) |
| 숫자 | 가중치 int8 × 입력 int16, 32비트 누산 |
| 클럭 | 148.33MHz |
| 메모리 통로 | Zynq HP 포트 2개 (64비트 AXI3), 엔진마다 하나 |
| 쓴 자원 | DSP 66/80 · LUT 23.65% · BRAM 20 |
| 타이밍 | WNS +0.339ns (148MHz 에서 여유 있음) |
| 결과 | 더블 레이첼 한 문장 **2.66초** (보드 CPU 만 쓰면 14.2초) · 노트북 CPU 와 **60/60 같은 답** |

---

## 3. 회로 구조

```
ARM Cortex-A9 ×2 (리눅스) — rachel_tpu
│  ① 가중치·입력을 TPU 전용 메모리에 놓는다
│  ② 레지스터(GP0, 0x43C00000)에 주소·크기를 쓰고 시작
▼
tpu_top (PL, 직접 설계한 회로)
├─ 엔진 0 (tpu_mv) ◀─ HP1 ─ DDR3
└─ 엔진 1 (tpu_mv) ◀─ HP2 ─ DDR3
│  ③ 결과를 메모리에 쓰면 rachel_tpu 가 읽어 간다
▼
DDR3 512MB 중 32MB (0x1C000000) = TPU 전용: 가중치 · 입력 · 결과
```

### 3-1. 엔진 하나 — `rtl/tpu_mv.v`

```
Y[행][m] = Σ  W[행][k] × X[m][k]        (m = 입력 벡터 1~4개, k = 최대 2048)
```

1. 먼저 입력 벡터 X(최대 4개, int16)를 칩 안 버퍼(BRAM)에 읽어 둔다.
2. 가중치 W 를 행 순서로 흘려 읽는다. 한 클럭에 64비트 = **int8 가중치 8개**.
3. 그 8개를 입력 벡터 4개와 동시에 곱한다 → 한 클럭에 **곱셈 32개** (DSP48).
4. 3단 덧셈 나무(8→4→2→1)로 더하고, 행마다 32비트로 누산한다.
5. 한 행이 끝나면 결과 16바이트(int32 × 4)를 메모리에 쓴다.

엔진이 2개라 **곱셈기 64개**. 한 행렬의 행을 두 엔진이 나눠 맡는다.

### 3-2. 왜 입력 벡터가 4개인가

말을 읽는 단계에서는 여러 글자를 한꺼번에 계산할 수 있다. 글자 4개를 한 번에 넣으면 **가중치를 한 번만 읽고 4번 쓴다.**
가중치 읽기가 가장 비싸기 때문에 이것만으로 읽는 단계가 크게 빨라진다.

### 3-3. 레지스터 — `rtl/tpu_top.v`

| 주소 (0x43C00000 + ) | 뜻 |
|---|---|
| 0x000 | ID = "DTPU" |
| 0x004 | 구성 (8칸 · 4벡터 · 엔진 2 · 판 1) |
| 0x008 / 0x00C | 전체 상태 / 여러 엔진 같은 클럭에 시작 |
| 0x100 + 0x40×엔진 | 시작 · 상태 · 가중치/입력/결과 주소 · K · 행 수 · 벡터 수 · 걸린 클럭 |

### 3-4. 넘침이 없게 — 숫자 범위를 먼저 정했다

입력 벡터는 곱하기 직전에 벡터마다 배율을 따로 정해 int16 으로 바꾼다 (|x| ≤ 8191).
그러면 가장 긴 곱셈(1792개 합)도 `1792 × 127 × 8191 < 2³¹` 이라 **32비트 누산이 절대 넘치지 않는다.**
이 규칙은 앞서 만든 정수 전용 엔진 DUSK(8장)에서 넘침 버그를 겪으며 배운 것이다.

---

## 4. 보드 위 프로그램 — `tpu/sw/rachel_tpu.cpp`, `tpu/sw/tpu_hw.h`

RACHEL C++ 엔진에 TPU 길을 더한 판이다. 커널 드라이버 없이 `/dev/mem` 으로 회로를 직접 부른다.

| 실행 방법 | 뜻 |
|---|---|
| `RACHEL_TPU=cpu` | 예전과 같이 CPU 로 |
| `RACHEL_TPU=emu` | TPU 와 **똑같은 정수 계산**을 CPU 로 (정확도 확인용) |
| `RACHEL_TPU=hw` | 보드의 TPU 로 |

- 행렬곱은 TPU, 나머지(LayerNorm·어텐션·GELU·softmax)는 ARM CPU 가 한다.
- 가중치는 한 번만 TPU 전용 메모리에 올린다. 여러 프로그램이 TPU 를 같이 쓰면 답 하나 단위로 차례를 지킨다 (파일 잠금 + 주인 표시).
- 빌드 (armhf, 정적): `g++ -O3 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -ffast-math -std=c++17 -static -pthread -DDUSK_TPU_HW -o rachel_tpu rachel_tpu.cpp`

### 4-1. 7.25초 → 2.66초 — 보드 프로그램을 다듬은 것

TPU 를 처음 붙였을 때는 문장당 7.25초였다. 회로는 그대로 두고 보드 프로그램을 고쳐 2.66초가 됐다.

| 고친 것 | 이유 |
|---|---|
| 말 읽는 단계를 글자 4개씩 묶음 | 가중치를 한 번 읽고 4번 씀 |
| 말 읽는 중간에는 다음 글자 계산(어휘 2,502개)을 건너뜀 | 답에 안 쓰이는 계산 |
| 입력을 int16 으로 바꾸는 계산을 벡터화 | 한 글자씩 반올림 함수를 부르면 느림 |
| TPU 메모리를 64바이트씩 한꺼번에 읽고 씀 | 캐시 없는 메모리라 작게 읽으면 느림 |
| KV 캐시를 헤드별로 배치 + fp16 | 어텐션이 메모리를 덜 읽음 |
| 어텐션을 NEON 으로만 계산 | Cortex-A9 는 NEON 과 일반 실수 연산을 번갈아 쓰면 매번 수십 클럭 멈춤 |
| GELU 를 빠른 근사로, GELU·어텐션을 2코어로 | |

남은 시간 (한 문장): TPU 기다림 1.1초 · 어텐션 0.6초 · 결과 읽기 0.3초 · GELU 0.18초.

### 4-2. 무엇이 속도를 정하나 — 메모리

답을 쓰는 단계에서는 **한 글자를 쓸 때마다 가중치 25MB 를 처음부터 끝까지 다 읽어야 한다.**
그래서 곱셈기를 더 늘려도 빨라지지 않고, 메모리 통로(HP 포트 2개, 실측 약 2.4GB/s)가 속도를 정한다.
곱셈기 64개(8×8)를 고른 이유가 이것이다. 이보다 늘려도 메모리가 못 따라온다.

---

## 5. 검증 — 같은 답이 나오는지 세 번 확인

| 단계 | 방법 | 결과 |
|---|---|---|
| ① 회로 시뮬레이션 | `sim/tb_tpu.v` (Icarus Verilog). 더블 레이첼의 실제 행렬 크기(448×1344, 448×448, 448×1792, 1792×448, 448×2502)와 입력 2~4개, 메모리 응답을 무작위로 늦추고 끊어 가며 계산 → 테스트벤치 안의 정답과 비교 | 8가지 전부 통과 |
| ② 같은 정수 계산을 CPU 로 | `RACHEL_TPU=emu` 로 60문장 | 원래 답과 **60/60 같음** |
| ③ 보드 실물 | `RACHEL_TPU=hw` 로 같은 60문장 | 노트북 CPU 답과 **60/60 같음** |

---

## 6. 결과

| 실행 | 한 문장 | 비고 |
|---|---|---|
| 보드 ARM CPU 2코어 | 14.2초 | 보드 프로그램 개선 뒤 |
| **보드 TPU** | **2.66초** | 글자 하나에 27~40ms |
| 노트북 CPU (참고) | 0.31초 | AMD 8845HS |

시연에서는 LattePanda Mu 가 ARM STUDIO 를 돌리고, 같은 문장을 DUSK_BOOM 이 TPU 로 **실제로 계산**한다.
보드의 HDMI 화면에는 TPU 가 지금 몇 번째 층을 계산하는지, 곱셈을 몇 번 했는지, 어떤 글자를 썼는지가 실시간으로 나온다.
보드 계산이 끝나는 순간 ARM STUDIO 화면에 블록이 붙는다 (한 문장 약 3~4초).

---

## 7. FPGA 로 한 나머지 일

### 7-1. 보드 리눅스 부팅

- 공장 SD 카드의 U-Boot 를 그대로 쓰고, 우리 파일만 더했다: `uEnv.txt` · `dusk.bit`(우리 회로) · `zImage`(커널) · `system.dtb`(장치 트리).
- 우리 부팅이 실패하면 공장 리눅스로 넘어가게 해 두었다. 되돌리기 = `uEnv.txt` 이름 바꾸기.
- 장치 트리에서 TPU 전용 32MB(0x1C000000)와 화면 4MB(0x1FC00000)를 떼어 리눅스가 건드리지 못하게 했다.
- 루트 파일 시스템은 SD 의 다른 칸에 Debian 12 (armhf).
- 비바도 배치 빌드: `vivado/build.tcl`, `vivado/run_build.bat` (비바도 2020.2).

### 7-2. HDMI 출력도 회로로 직접

HDMI 영상 신호(TMDS)를 FPGA 회로로 직접 만든다.

| 파일 | 하는 일 |
|---|---|
| `rtl/hdmi_timing.v` | 1280×720 60Hz 타이밍 (픽셀 클럭 74.25MHz) |
| `rtl/fb_scanout.v` | 리눅스가 메모리(DDR)에 그린 화면을 HP0 포트로 읽어 옴. 줄 버퍼 2개로 한 줄 보여 주는 동안 다음 줄을 읽음 |
| `rtl/tmds_encoder.v` · `rtl/tmds_serializer.v` | HDMI(TMDS) 부호화 · 고속 직렬화 |
| `rtl/clkgen.v` | 픽셀·TPU 클럭 만들기 (MMCM) |

리눅스는 이 메모리를 보통 화면(`/dev/fb0`)으로 쓴다. 그래서 콘솔도, 시연 화면도 그대로 HDMI 로 나간다.

### 7-3. TPU 시연 화면 — `tpu_show/tpu_show.py`

- 보드에서 pygame 으로 `/dev/fb0` 에 직접 그린다 (브라우저 없음). 바뀌지 않는 배경은 미리 그려 두어 한 장에 37ms.
- 보드 프로그램이 계산하는 동안 층·곱셈 수·글자를 기록(`RACHEL_TRACE`)하면 화면이 그것을 읽어 그린다.
- LattePanda Mu 와 보드는 USB 시리얼 한 가닥으로 연결: Mu 가 문장을 보내면 보드가 계산하고 끝났다고 알린다 (`mu/dusk_tpu/tpu_bridge.py`).

### 7-4. 로봇을 보드 USB 로 연결하는 설계

같은 보드로 ESP32-S3(와이파이) → FPGA → 리눅스 → USB-A → 로봇 서보로 이어지는 연결도 설계했다 (FPGA 시뮬레이션 52항목 통과).
→ `dusk_boom_fpga_v35/README.md`

---

## 8. 앞선 단계 — 정수 전용 엔진 DUSK

TPU 전에, FPU(실수 계산기)가 없는 옛 CPU(68000) 여러 개를 묶어 GPU 처럼 쓰는 장치를 목표로 RACHEL 추론을 **실수 연산 0줄**로 다시 짰다 (`Industry_gyu_arm/dusk/`).

- 행렬곱·LayerNorm·잔차·GELU·어텐션·softmax 까지 전부 정수 곱셈·시프트·표 찾기.
- 같은 200문장에서 실수판과 같은 **197/200**.
- 잡은 버그는 전부 숫자 범위 문제였다: 32비트 누산 넘침, 배율 이중 나눗셈, KV 캐시에 그때의 배율을 안 남긴 것.
- 방법: 정답(실수)에서 출발해 **한 단계씩** 정수로 바꾸면 틀린 곳이 바로 나온다.
- 여기서 배운 "계산마다 배율을 따로 정하고, 넘침이 없는 범위를 먼저 증명한다" 를 TPU 설계(3-4)에 그대로 썼다.
- 68000 실물 칩으로는 아직 돌리지 않았다.

---

## 9. 파일 위치

| 무엇 | 어디 (`dusk_boom_fpga_v35/` 아래) |
|---|---|
| TPU 회로 | `rtl/tpu_mv.v` (엔진) · `rtl/tpu_top.v` (엔진 2개 + 레지스터) |
| 최상위 회로 · HDMI · 클럭 | `rtl/dusk_boom_v35_top.v` · `rtl/fb_scanout.v` · `rtl/hdmi_timing.v` · `rtl/tmds_*.v` · `rtl/clkgen.v` |
| 핀 · 타이밍 제약 | `constraints/` |
| 시뮬레이션 | `sim/tb_tpu.v` (TPU) · `sim/tb_fb.v` (화면) · `sim/tb_v35.v` (보드 전체) |
| 비바도 빌드 | `vivado/` |
| 보드 프로그램 (TPU 판 RACHEL) | `tpu/sw/rachel_tpu.cpp` · `tpu/sw/tpu_hw.h` · `tpu/sw/bench/` |
| 시연 화면 | `tpu_show/` |
| 보드 리눅스 | `linux/` |
| LattePanda Mu 쪽 연결 | `mu/` |
| 정수 전용 엔진 DUSK | `Industry_gyu_arm/dusk/` |

---

## 10. 다시 만들기

```bash
# 1) TPU 회로 시뮬레이션 (Icarus Verilog)
cd dusk_boom_fpga_v35/sim
iverilog -g2005 -o tb_tpu.vvp ../rtl/tpu_mv.v ../rtl/tpu_top.v tb_tpu.v
vvp -n tb_tpu.vvp

# 2) 비트스트림 (Windows, 비바도 2020.2 — 경로에 한글·빈칸이 없게)
vivado\run_build.bat

# 3) 보드 프로그램 (armhf)
cd dusk_boom_fpga_v35/tpu/sw
g++ -O3 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -ffast-math -std=c++17 -static -pthread \
    -DDUSK_TPU_HW -o rachel_tpu rachel_tpu.cpp

# 4) 보드에서: 같은 문장을 TPU 와 CPU 로
RACHEL_TPU=hw  ./rachel_tpu double.bin "$(cat prompt1.txt)"
RACHEL_TPU=cpu ./rachel_tpu double.bin "$(cat prompt1.txt)"
```

가중치 `double.bin` 은 저장소에 없다 → [RACHEL.md](RACHEL.md) 8장으로 만든다.
