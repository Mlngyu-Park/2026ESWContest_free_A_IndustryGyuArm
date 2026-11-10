# DUSK_BOOM TPU — 작동 원리와 비바도로 만드는 법

Zynq XC7Z010 FPGA 에 Verilog 로 직접 설계한 **행렬곱 전용 TPU** 다.
더블 레이첼(25.6MB, 파라미터 2,560만 개) 추론에서 시간이 가장 많이 드는 "가중치 행렬 × 입력 벡터" 를 이 회로가 맡는다.

| 항목 | 값 |
|---|---|
| 칩 | Xilinx Zynq XC7Z010 (`xc7z010clg400-1`, ALINX AC7010C 모듈) |
| 곱셈기 | 엔진 2개 × (가중치 8개 × 입력 벡터 4개) = **64개** (DSP48) |
| 숫자 | 가중치 int8 × 입력 int16 → 32비트 누산 |
| 클럭 | 148.33MHz |
| 메모리 통로 | 엔진마다 Zynq HP 포트 하나 (64비트 AXI3) |
| 자원 (TPU 만) | LUT 3,575 · FF 3,603 · BRAM36 16 · DSP 66 |
| 타이밍 | WNS +0.339ns (통과) |
| 결과 | 더블 레이첼 한 문장 2.66초 (보드 CPU 만 14.2초) · 노트북 CPU 와 60/60 같은 답 |

---

## 1. TPU 가 하는 일

언어 모델이 글자 하나를 쓸 때마다 하는 계산의 대부분은 이것이다.

```
y = W · x        W: 가중치 행렬 (행 × K, int8)   x: 입력 벡터 (K, int16)   y: 결과 (행, int32)
```

- 행렬의 **각 행은 서로를 기다릴 필요가 없다.** 그래서 곱셈기를 여러 개 놓고 동시에 돌리면 빨라진다.
- 더블 레이첼에서 쓰는 행렬 크기: 448×1344 (어텐션 qkv), 448×448, 448×1792 · 1792×448 (MLP), 448×2502 (다음 글자 고르기).
- 나머지 계산(LayerNorm, 어텐션 점수, GELU, softmax)은 보드의 ARM CPU 가 한다.

---

## 2. 작동 원리

### 2-1. 전체 연결

```
ARM Cortex-A9 ×2 (리눅스) — rachel_tpu 프로그램
│  ① 가중치·입력을 TPU 전용 메모리(DDR 의 32MB)에 놓는다
│  ② 레지스터(M_AXI_GP0, 0x43C00000)에 주소·크기를 쓰고 시작
▼
tpu_top (PL = 직접 설계한 회로, 148.33MHz)
├─ 엔진 0 (tpu_mv) ◀─ S_AXI_HP1 (64비트) ─ DDR3
└─ 엔진 1 (tpu_mv) ◀─ S_AXI_HP2 (64비트) ─ DDR3
│  ③ 결과를 메모리에 쓰고 '끝' 표시
▼
rachel_tpu 가 결과를 읽어 실수로 되돌린다
```

- **GP0** (CPU → 회로): 레지스터 읽기·쓰기 통로.
- **HP1 · HP2** (회로 → 메모리): 회로가 DDR 을 직접 읽고 쓰는 고속 통로. 엔진마다 하나씩.
- HP0 은 HDMI 화면 회로가 쓴다.

### 2-2. 엔진 하나 안에서 — `rtl/tpu_mv.v`

**준비**: 입력 벡터 n개(1~4, int16)를 먼저 읽어 칩 안 메모리(BRAM)에 넣어 둔다. 벡터 하나에 최대 2,048개.

**본 계산**: 가중치를 행 순서대로 흘려 읽는다. 128바이트 묶음으로 요청하고, 최대 8묶음을 미리 요청해 둔다.
매 클럭 64비트 = **int8 가중치 8개**가 도착하고, 아래 단계를 한 칸씩 지나간다 (매 클럭 새 값이 들어오는 파이프라인).

| 단계 | 하는 일 |
|---|---|
| s1 | 도착한 가중치 8개를 레지스터에 잡고, 같은 클럭에 BRAM 에 "입력의 몇 번째 8개" 를 요청 |
| s2 | BRAM 출력을 기다림 (가중치도 한 칸 대기) |
| s3 | 가중치와 입력을 곱셈기(DSP) 바로 앞 레지스터에 나란히 둠 |
| s4 | **곱셈 32개 동시**: 가중치 8개 × 입력 벡터 4개 (DSP48) |
| s5~s7 | 벡터마다 덧셈 나무 8 → 4 → 2 → 1 |
| s8 | 32비트로 누산. 행의 마지막이면 결과 4개(int32 × 4 = 16바이트)를 출력 FIFO 로 |

**쓰기**: 출력 FIFO 에서 8행(128바이트)씩 묶어 HP 포트로 메모리에 쓴다.
FIFO 가 차면 가중치 받기를 잠깐 멈춘다 (흐름 제어).

엔진 하나 = 매 클럭 곱셈 32개. 엔진 둘이 행렬의 행을 반씩 나눠 맡는다 → **곱셈기 64개**.

### 2-3. 왜 입력 벡터가 4개인가

말을 읽는 단계에서는 여러 글자를 한꺼번에 계산할 수 있다. 글자 4개를 한 번에 넣으면 **가중치를 한 번 읽고 4번 쓴다.**
TPU 에서 가장 비싼 것은 곱셈이 아니라 가중치를 메모리에서 읽는 일이기 때문이다.

### 2-4. 무엇이 속도를 정하나 — 메모리

- 엔진 하나가 매 클럭 8바이트를 읽는다: 148MHz × 8B ≈ 1.19GB/s, 엔진 둘이면 약 2.4GB/s.
- 답을 쓰는 단계에서는 글자 하나마다 가중치 25MB 를 전부 읽어야 한다 → 글자 하나에 약 27~40ms.
- 곱셈기를 더 늘려도 메모리가 못 따라와서 빨라지지 않는다. 그래서 8×8 = 64개에서 멈췄다.

### 2-5. 숫자가 넘치지 않게 — 먼저 증명

입력은 곱하기 직전에 벡터마다 배율을 따로 정해 int16 으로 바꾼다: `|x| ≤ 8191`.
가장 긴 행(1,792개 합)도 `1792 × 127 × 8191 < 2³¹` 이라 **32비트 누산이 절대 넘치지 않는다.**
결과는 프로그램이 `가중치 배율 × 입력 배율` 을 곱해 실수로 되돌린다.

### 2-6. 레지스터 — `rtl/tpu_top.v`

| 주소 (0x43C00000 + ) | 이름 | 뜻 |
|---|---|---|
| 0x000 | ID | "DTPU" (0x44545055) |
| 0x004 | CONFIG | 8칸 · 4벡터 · 엔진 2 · 판 1 |
| 0x008 | GSTATUS | 엔진별 일하는 중 / 끝 |
| 0x00C | GSTART | 고른 엔진을 **같은 클럭에** 시작 |
| 0x100 + 0x40×e | 엔진 e | +0x08 가중치 주소 · +0x0C 입력 주소 · +0x10 결과 주소 · +0x14 K/8 · +0x18 행 수 · +0x1C 벡터 수 · +0x20 걸린 클럭 |

### 2-7. 프로그램에서 한 번 곱하기 — `tpu/sw/tpu_hw.h`

```
1. (처음 한 번) 가중치를 TPU 전용 메모리에 복사 — 텐서마다 4KB 정렬
2. 입력 벡터 n개를 int16 으로 바꿔 입력 버퍼에 쓴다
3. 행을 반으로 나눈다 (엔진 0 이 앞 절반, 엔진 1 이 뒤 절반)
4. 엔진 0·1 레지스터에 주소 · K/8 · 행 수 · 벡터 수를 쓴다
5. GSTART = 3  → 두 엔진 동시 시작
6. GSTATUS 가 0 이 될 때까지 기다린다
7. 결과(int32)를 읽어 배율을 곱하고 bias 를 더한다
```

커널 드라이버 없이 `/dev/mem` 으로 레지스터와 메모리를 직접 연다. TPU 전용 메모리는 리눅스가 쓰지 못하게 장치 트리에서 떼어 둔다 (5장).

---

## 3. 비바도로 만드는 법

### 3-0. 준비물

| | |
|---|---|
| 비바도 | 2020.2 (Windows). 2019.2~2024.x 에서 돌도록 썼다 |
| 부품 | `xc7z010clg400-1` |
| 소스 | `dusk_boom_fpga_v35/` 폴더 전체 (`rtl/` · `constraints/` · `vivado/`) |
| 주의 | **경로에 한글·빈칸이 없게** (예: `C:\work\fpga_v35`). 비바도는 한글 경로에서 멈춘다 |

### 3-1. 한 번에 만들기

```bat
:: Windows — vivado\run_build.bat 과 같다
cd /d C:\work\fpga_v35\vivado
call C:\Xilinx\Vivado\2020.2\settings64.bat
vivado -mode batch -nojournal -log build.log -source build.tcl
```

노트북에서 10분 안팎. 끝나면 `vivado/` 에:

| 파일 | 뜻 |
|---|---|
| `dusk_boom_v35_top.bit` | 비트스트림 (FPGA 회로) |
| `dusk_boom_v35_top.bin` | 리눅스 fpga_manager 용 같은 회로 |
| `system.xsa` | 하드웨어 플랫폼 (PS 설정 + 비트스트림) |
| `reports/` | 타이밍 · 자원 · 클럭 · CDC · DRC 보고서 |

마지막 줄에 `WNS = … ns   WHS = … ns` 가 나온다. **둘 다 0 이상**이어야 한다.

GUI 로 열어 보고 싶으면 블록 디자인까지만 만든다:
```
vivado -mode batch -source build.tcl -tclargs project_only
→ vivado/build/dusk_boom_v35.xpr 를 GUI 로 열기
```

### 3-2. `build.tcl` 이 하는 일 — GUI 로 직접 할 때도 이 순서

**① 프로젝트 만들기**
- 부품 `xc7z010clg400-1`, 언어 Verilog.

**② 블록 디자인에 Zynq PS 넣기** (`system`)
- `processing_system7` 하나만 넣는다. DDR · FIXED_IO 는 밖으로.
- AC7010C 설정을 `vivado/ps7_ac7010c.tcl` 로 입힌다:
  - DDR3: MT41J128M16 × 2 = **512MB**, 32비트, 533MHz (1GB 판인 AC7020C 의 MT41K256M16 이 아님)
  - CPU 666MHz, 크리스털 33.33MHz, UART · USB · 이더넷 · SD · QSPI 의 MIO 핀
  - **M_AXI_GP0 켜기** (TPU 레지스터)
  - **S_AXI_HP1 · S_AXI_HP2 켜기, 64비트** (TPU 엔진 2개)
  - S_AXI_HP0 켜기, 64비트 (HDMI 화면)

**③ TPU 통로를 블록 디자인 밖으로 꺼내기**
- `M_AXI_GP0` → 포트 `tpu_ctl`
- `S_AXI_HP1` → 포트 `tpu_m0`, `S_AXI_HP2` → 포트 `tpu_m1`
- 클럭 입력 포트 `tpu_aclk` (148,333,333Hz) 를 만들어 `M_AXI_GP0_ACLK` · `S_AXI_HP1_ACLK` · `S_AXI_HP2_ACLK` 에 연결
- `tpu_aclk` 의 ASSOCIATED_BUSIF = `tpu_ctl:tpu_m0:tpu_m1`

**④ 주소 정하기**
- Address Editor 에서 `tpu_ctl` = **0x43C00000, 64KB**.

**⑤ wrapper 만들고 회로 넣기**
- Validate → Generate Output Products → Create HDL Wrapper (`system_wrapper`).
- `rtl/*.v` 전부 추가, 최상위 = `dusk_boom_v35_top`.
- 최상위가 `system_wrapper` 의 `tpu_ctl_*` · `tpu_m0_*` · `tpu_m1_*` 를 `tpu_top` 에 잇는다 (`rtl/dusk_boom_v35_top.v`).

**⑥ 제약 파일**
- `constraints/dusk_boom_v35_pins.xdc` (핀), `clock.xdc` (50MHz 입력), `timing_v35.xdc` (클럭 사이 관계, 구현에만 사용).

**⑦ 합성 · 구현 설정**
- 합성: `Flow_PerfOptimized_high`, **`MAX_DSP = 66`** — 7010 의 DSP 80개 중 곱셈기 64개 + 화면 1개만 쓰게.
- 구현: `Performance_Explore`.

**⑧ 합성 → 구현 → 비트스트림 → 보고서 → .bin · .xsa**

### 3-3. TPU 클럭 만들기 — `rtl/clkgen.v`

PS 의 FCLK 를 쓰지 않고, 모듈의 50MHz 발진기(PL 핀 U18)에서 MMCM 으로 만든다.

```
50MHz ÷ 3 × 44.5 = 741.67MHz (MMCM 안)
   ÷ 2  = 370.83MHz  HDMI 직렬화
   ÷ 10 =  74.17MHz  HDMI 픽셀
   ÷ 5  = 148.33MHz  TPU  ← clk_tpu (GP0 · HP1 · HP2 의 AXI 클럭도 이것)
```

### 3-4. 합성기에게 알려 준 것 (Verilog 속성)

| 위치 | 속성 | 이유 |
|---|---|---|
| 곱셈 32개 `p[]` | `use_dsp = "yes"` | 곱셈은 DSP48 에 |
| 덧셈 나무 `a3 a4 a5`, 누산 `acc`, `pushd` | `use_dsp = "no"` | 덧셈까지 DSP 를 쓰면 80개가 모자란다 |
| 입력 버퍼 `tpu_ram256x128` | `ram_style = "block"` | 128비트 × 256 × 4개는 BRAM 에 |
| 출력 FIFO | `ram_style = "distributed"` | 작고 빨라야 해서 LUT 메모리에 |

---

## 4. 타이밍을 맞춘 과정 — 비바도 빌드 4번

148.33MHz 면 한 클럭이 6.74ns 다. 모든 길이 이 안에 끝나야 한다 (WNS ≥ 0).

| 판 | 바꾼 것 | 결과 |
|---|---|---|
| 1차 | 처음 설계 그대로 | 덧셈까지 DSP 로 가서 **DSP 80/80** 을 다 씀 → **배치 실패** (`Placer could not place all instances`) |
| 2차 | 덧셈·누산은 DSP 금지(`use_dsp="no"`), 합성 `MAX_DSP 66`, 입력 버퍼에 BRAM · 출력 FIFO 에 LUT 메모리 속성 | DSP 66, 배치 성공. 그러나 **WNS −2.24ns**. 입력 버퍼는 속성만으로는 BRAM 이 안 됨 (BRAM36 4개) |
| 3차 | 긴 길 끊기: 읽을 길이 계산(곱셈)을 미리 레지스터에 담음 · 4KB 경계 계산을 없애고 "16묶음 + 나머지 한 번" 으로 (프로그램이 128바이트 정렬을 보장) · 입력 버퍼를 **BRAM 전용 모듈**(`tpu_ram256x128`)로 따로 뺌 | 입력 버퍼가 실제로 BRAM 으로 (BRAM36 20개). **WNS −1.16ns** |
| 4차 | BRAM 읽기를 2클럭으로 (출력 레지스터 사용) · 가중치를 2칸 더 늦춰 입력과 **곱셈기 바로 앞 레지스터에서** 만나게 · FIFO 여유 20 → 24칸 | **WNS +0.339ns 통과** |

배운 것: 늦은 길은 **레지스터를 하나 더 넣어 둘로 나누면** 풀린다. 대신 파이프라인이 길어지므로, 늘어난 칸만큼 FIFO 여유도 늘려야 결과가 넘치지 않는다.

최종 자원 (보드 전체): LUT 4,163 (23.65%) · FF 4,159 (11.82%) · BRAM 20/60 · **DSP 66/80**.

---

## 5. 보드에 올리기

### 5-1. SD 카드

보드의 공장 SD 카드에 있는 U-Boot 를 그대로 쓰고, 첫 칸(FAT)에 4개를 더한다.

| 파일 | 어디서 |
|---|---|
| `dusk.bit` | `vivado/dusk_boom_v35_top.bit` 이름만 바꿈 |
| `zImage` | 리눅스 커널 (`linux/build_linux.sh`) |
| `system.dtb` | 장치 트리 (`linux/dts/zynq-dusk-boom.dts` 를 dtc 로) |
| `uEnv.txt` | 아래 |

```
dusk_boot=fatload mmc 0 0x10000000 dusk.bit && fpga loadb 0 0x10000000 ${filesize} && fatload mmc 0 0x2000000 system.dtb && fatload mmc 0 0x2080000 zImage && setenv bootargs console=tty0 console=ttyPS0,115200 earlycon && bootz 0x2080000 - 0x2000000
bootcmd=run dusk_boot
uenvcmd=boot
```

- U-Boot 가 우리 회로(`dusk.bit`)를 FPGA 에 올린 뒤 리눅스를 켠다.
- 공장 U-Boot 는 `test -n $uenvcmd` 로 검사하므로 `uenvcmd` 는 **한 단어**여야 한다.
- 실패하면 공장 리눅스로 넘어간다. 되돌리기 = `uEnv.txt` 이름 바꾸기.
- Vitis 가 없어 FSBL 은 공장 것을 쓴다.

### 5-2. TPU 전용 메모리 떼어 두기 — 장치 트리

`linux/dts/zynq-dusk-boom.dts` 에서 발췌:

```dts
reserved-memory {
    tpu_mem: tpu@1c000000 {
        reg = <0x1c000000 0x2000000>;   /* 32MB: 가중치 25.6MB + 입력·결과 버퍼 */
        no-map;
    };
    screen_mem: screen@1fc00000 {
        reg = <0x1fc00000 0x400000>;    /* 4MB: HDMI 화면 1280×720×4 */
        no-map;
    };
};
```

`no-map` 이라 리눅스가 이 메모리를 절대 쓰지 않는다. TPU 와 프로그램만 쓴다.

| 0x1C000000 + | 쓰임 |
|---|---|
| 0x0000000 ~ | 가중치 (텐서마다 4KB 정렬, 행 순서 int8) |
| 0x1E00000 | 입력 버퍼 (int16) |
| 0x1E80000 | 엔진 0 결과 |
| 0x1F00000 | 엔진 1 결과 |

### 5-3. 보드 프로그램 빌드 · 실행

```bash
cd dusk_boom_fpga_v35/tpu/sw
g++ -O3 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -ffast-math -std=c++17 -static -pthread \
    -DDUSK_TPU_HW -o rachel_tpu rachel_tpu.cpp

RACHEL_TPU=hw  ./rachel_tpu double.bin "<장면>\n<말>"     # TPU
RACHEL_TPU=cpu ./rachel_tpu double.bin "<장면>\n<말>"     # 비교: CPU 만
RACHEL_TPU=emu ./rachel_tpu double.bin "<장면>\n<말>"     # TPU 와 똑같은 정수 계산을 CPU 로
```

---

## 6. 검증 — 같은 답이 나오는지 세 번

| 단계 | 방법 | 결과 |
|---|---|---|
| ① 회로 시뮬레이션 | `sim/tb_tpu.v` (Icarus Verilog). 더블 레이첼의 실제 행렬 크기 × 입력 2~4개, 메모리 응답을 무작위로 늦추고 끊어 가며 → 테스트벤치 안의 정답과 비교 | 8가지 전부 통과 |
| ② 정수 계산 흉내 | `RACHEL_TPU=emu`, 60문장 | 원래 답과 60/60 같음 |
| ③ 보드 실물 | `RACHEL_TPU=hw`, 같은 60문장 | 노트북 CPU 답과 **60/60 같음** |

```bash
cd dusk_boom_fpga_v35/sim
iverilog -g2005 -o tb_tpu.vvp ../rtl/tpu_mv.v ../rtl/tpu_top.v tb_tpu.v
vvp -n tb_tpu.vvp
```

---

## 7. 결과

| 실행 | 더블 레이첼 한 문장 |
|---|---|
| 보드 ARM CPU 2코어 | 14.2초 |
| **보드 TPU** | **2.66초** |

TPU 를 처음 붙였을 때는 7.25초였다. 회로는 그대로 두고 보드 프로그램을 다듬어 2.66초가 됐다 (말 읽기 4글자 묶음, 캐시 없는 메모리를 64바이트씩 읽고 쓰기, 어텐션 NEON 전용, GELU·어텐션 2코어).
남은 시간: TPU 기다림 1.1초 · 어텐션 0.6초 · 결과 읽기 0.3초 · GELU 0.18초.

전력: 비바도 전력 분석 기준 기본 1.96W + TPU 가동분 0.55W → **최대 약 2.5W**.

---

## 8. 파일

| 무엇 | 어디 (`dusk_boom_fpga_v35/` 아래) |
|---|---|
| TPU 엔진 | `rtl/tpu_mv.v` |
| 엔진 2개 + 레지스터 | `rtl/tpu_top.v` |
| 최상위 (PS ↔ TPU · 화면 연결) | `rtl/dusk_boom_v35_top.v` |
| 클럭 | `rtl/clkgen.v` |
| 핀 · 타이밍 제약 | `constraints/` |
| 비바도 빌드 · PS 설정 | `vivado/build.tcl` · `vivado/ps7_ac7010c.tcl` · `vivado/run_build.bat` |
| TPU 시뮬레이션 | `sim/tb_tpu.v` |
| 장치 트리 | `linux/dts/zynq-dusk-boom.dts` |
| 보드 프로그램 · 드라이버 | `tpu/sw/rachel_tpu.cpp` · `tpu/sw/tpu_hw.h` |
| 시연 화면 | `tpu_show/tpu_show.py` |

가중치 `double.bin` 은 저장소에 없다 → [RACHEL.md](RACHEL.md) 8장으로 만든다. 보드 전체 이야기는 [DUSK_BOOM.md](DUSK_BOOM.md).
