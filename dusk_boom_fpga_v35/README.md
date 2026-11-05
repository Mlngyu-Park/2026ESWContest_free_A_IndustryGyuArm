# DUSK_BOOM 로봇판 v3.5 — FPGA 모듈 USB 로 로봇 연결 (2026-11-05)

로봇(페테크 버스 서보 + USB 어댑터)을 **AC7010C 모듈의 USB-A** 에 꽂아 움직이는 설계 한 벌이다.
비바도 코드 · 리눅스 · ESP 펌웨어 변경 · 시험 순서가 이 폴더에 다 있다.

## 1. 어떻게 이어지나

```
휴대폰/PC ─와이파이─ ESP32-S3 ─(ESP_IO1 → FPGA)─ Zynq UART0 ─ 리눅스 dusk_robotd ─ 모듈 USB-A ─ 서보 어댑터 ─ 서보
                         └──(ESP_IO0 ← FPGA)── 답 ──────────────┘
```

- **ESP 는 하던 대로** 페테크 명령을 만든다. 보내는 길만 하나 늘었다: `LINK FPGA`.
- FPGA(PL)는 ESP_IO0/1 두 가닥을 Zynq 의 ARM 쪽(PS) UART0 에 **선으로 그냥 잇는다**. 속도는 1Mbps.
- ARM 쪽 리눅스의 `dusk_robotd` 가 그 UART 와 USB 어댑터(`/dev/ttyACM0` · `/dev/ttyUSB0`) 사이에서 바이트를 그대로 옮긴다.
- USB 호스트는 ARM 쪽 기능이라 **리눅스가 꼭 필요**하다. 맨 FPGA 만으로는 USB 장치를 못 다룬다.
- 그래서 SD 카드 한 장에 넣는다: 부팅 파일 + FPGA 비트스트림 + U-Boot + 리눅스 커널(파일 시스템 포함) + 장치 트리.

## 2. 폴더

| 무엇 | 어디 |
|---|---|
| FPGA 코드 (PL) | `rtl/` — `dusk_boom_v35_top.v` (최상위) · `spi_regs_v35.v` · `esp_io_v35.v` · 나머지는 `../fpga/rtl` 에서 그대로 복사 |
| 핀 · 타이밍 | `constraints/` — `dusk_boom_v35_pins.xdc` (로봇판 v3.5 xdc 그대로, 카메라 클럭 한 줄만 뺌) · `clock.xdc` · `timing_v35.xdc` |
| 비바도 | `vivado/build.tcl` (프로젝트 + PS 블록 + 비트스트림 + system.xsa) · `ps7_ac7010c.tcl` (ARM 쪽 설정, 모듈 매뉴얼 값) · `make_boot.tcl` (FSBL + BOOT.bin) · `build_all.bat` / `build_all.sh` |
| 시뮬 · 합성 확인 | `sim/run_sim.sh` (아이베릴로그 52항목) · `sim/run_yosys.sh` (요시스 합성) |
| 리눅스 | `linux/build_linux.sh` (스파크에서 U-Boot · 커널 · 파일 시스템 빌드 → `sd/`) · `linux/dts/` · `linux/robotd/` (dusk_robotd · dusk_servo · dusk_esp) · `linux/qemu_test.py` · `linux/make_sd_from_xsa.sh` |
| ESP 변경 | `esp_patch/esp32s3_robot_fpga_link.patch` (원본 펌웨어엔 아직 안 넣음) · `esp_patch/build/` (패치 넣고 빌드한 굽기 파일) |

## 3. 레지스터 · 핀 약속 (v3.5 에서 바뀐 것)

SPI 레지스터 (SPI_PROTOCOL.md 그대로 + 아래)
- `0x01` VERSION = **0x35**
- `0x08` STATUS: bit0 HDMI 모니터 · bit2 HDMI 클럭 잠김 · **bit3 로봇 다리 켜짐**(리눅스 dusk_robotd 돎) · **bit4 로봇 어댑터 열림** · bit5/6 앞판 버튼
- `0x09` FEATURES = 0x01 (로봇 다리 있음). v2.3 비트스트림은 0
- `0x22` ESP_IO_DIR: bit0·1 은 못 씀 (다리 몫). 읽으면 bit0 = FPGA 가 ESP_IO0 을 모는 중
- 카메라 레지스터(0x0C~0x13)는 없어짐 (0 으로 읽힘)

ESP_IO0 (ESP IO4) = FPGA → ESP (ESP 받기) · ESP_IO1 (ESP IO5) = ESP → FPGA (ESP 보내기). 다리가 꺼져 있으면 FPGA 는 ESP_IO0 을 안 몬다.

ARM 쪽 GPIO (EMIO, 리눅스 번호 = zynq_gpio 시작 + 54 + n)
- 0 ESP 리셋 · 1 ESP 굽기 모드 · 2/3 앞판 LED · 4/5 앞판 버튼 · 6 로봇 어댑터 열림 · 7 다리 켜짐
- 켜진 직후엔 전부 0 → FPGA 가 ESP 를 리셋에 묶지 않는다

모듈 LED 두 개
- **PS LED (MIO0)**: 심장 박동처럼 깜빡 = 리눅스 살아 있음
- **PL LED (R19)**: 느리게 깜빡 = 비트스트림만 올라감 · 빠르게 깜빡 = 다리 켜짐 · **계속 켜짐 = 로봇 어댑터 열림**

## 4. 아침 순서

### 4-0. 리눅스 파일 만들기 (스파크, 한 번만, 약 20분)

```
cd ~/project/2026_project/kihd_hotel/output/DUSK_BOOM/공통/fpga_v35/linux
./build_linux.sh            # → ../sd/ 에 u-boot.elf · zImage · system.dtb
python3 qemu_test.py ../sd  # (선택) 가상 Zynq 로 부팅 + 가짜 서보 시험
```
소스는 이미 다 받아 둠 (`~/opt/dusk_zynq_linux/`). CPU 는 10개까지만 씀.

### 4-1. 비바도 (x86 PC)

이 폴더(`fpga_v35`) 전체를 PC 로 복사한다. **경로에 한글 · 빈칸이 없게** (예: `C:\work\fpga_v35`) — 비바도는 한글 경로에서 멈춘다.
```
C:\work\fpga_v35\vivado\build_all.bat      (더블 클릭. 리눅스 PC 면 ./build_all.sh)
```
`build_all.bat` 이 `C:\Xilinx\Vivado\…` · `C:\Xilinx\Vitis\…` 를 스스로 찾는다. 다른 곳에 깔았으면 그 `settings64.bat` 를 먼저 실행.
- 끝나면 `vivado/dusk_boom_v35_top.bit` · `vivado/system.xsa` · `reports/`. **WNS · WHS 가 0 이상**이어야 한다
- Vitis(`xsct`) 가 있으면 이어서 FSBL 을 만들고 `sd_card/` 에 SD 파일이 다 모인다: `BOOT.bin` · `zImage` · `system.dtb`
- Vitis 가 없으면 `system.xsa` 를 스파크로 보내 `linux/make_sd_from_xsa.sh system.xsa` → `sd_card_spl/` (FSBL 대신 U-Boot SPL 로 같은 일을 함)

### 4-2. SD 카드 · 점퍼

1. microSD 첫 파티션을 **FAT32** 로 → `sd_card/` 의 파일을 맨 위 폴더에 그대로
2. 모듈 **J13 = SD** (매뉴얼 표: 왼쪽 두 핀. 실크 인쇄로 확인)
3. 모듈 **J5 · J6 점퍼 캡 끼우기 = USB 호스트** ← 이게 없으면 USB-A 가 장치를 못 알아본다
4. 로봇 어댑터 → **모듈 USB-A**
5. 모듈 마이크로 USB(USB-UART) → PC, 터미널 **115200**

### 4-3. 켜고 확인

1. 전원 → 터미널에 U-Boot → 리눅스 → `DUSK_BOOM robot v3.5 — Zynq Linux` 안내가 나옴
2. 모듈 PS LED 심장 박동 · PL LED 빠르게 깜빡 → 어댑터가 잡히면 계속 켜짐
3. 콘솔에서 `dusk_servo scan` → `ID 1 answers ... position=...` 이 서보 수만큼 (핑 · 읽기만, **안 움직임**)
4. `dusk_esp status` · `cat /var/log/dusk_robotd.log`

### 4-4. ESP 로 (와이파이)

1. ESP 굽기: `esp_patch/build/` 의 파일. **JP1 캡 빼고** YD 보드의 COM 쪽 USB-C 로
   ```
   cd esp_patch/build
   python -m esptool --chip esp32s3 -p <포트> -b 460800 write_flash "@flash_args"
   ```
   (원본 펌웨어에 패치를 넣어 직접 빌드하려면: `cd 공통/firmware/esp32s3_robot && patch -p1 < ../../fpga_v35/esp_patch/esp32s3_robot_fpga_link.patch && idf.py build`)
2. UDP: `FREG 1` → **0x35** · `FREG 9` → 0x01 · `STATUS` → `bridge=running(robot-open)`
3. `LINK FPGA` → `SPING 1` · `SREAD 1` (읽기만)
4. 서보를 움직이는 `SERVO <id> <pos>` 는 직접 판단해서

JTAG 만 있을 때: `.bit` 만 올리면 PL 만 돈다 → `FREG 0` = 0xB0 · `FREG 1` = 0x35 는 되지만 리눅스가 없어서 로봇 다리는 꺼져 있다.

## 5. 확인한 것 · 못 한 것

확인함
- FPGA 시뮬 52항목 통과: ID · VERSION · FEATURES · STATUS 비트 · ESP_IO_DIR 잠금 · 다리 켜짐/꺼짐 때 ESP_IO0 높은 임피던스 · 1Mbps 양방향 6바이트씩 · ESP 리셋/굽기 · LED · 버튼
- 요시스 합성 통과 (문제 0, 논리 약 284 · BUFG 4 · MMCM 1 · OSERDES 8)
- top 포트 = v3.5 xdc 포트 (빠짐 · 남음 0)
- dusk_robotd · dusk_servo PC 시험 전부 통과 (가짜 ESP · 가짜 서보 5개: 핑 · 위치 읽기 · TCP · 서보 도구 · 연속 5개 4.6ms · 어댑터 뽑힘)
- ESP 패치: 호스트 시험 205개 통과 · ESP-IDF 빌드 통과 (경고 0)

못 함 (실물 · 다른 PC 필요)
- **비바도로 실제 돌린 적 없음** (스파크는 ARM 이라 비바도가 안 돈다). PS7 설정 이름은 디질런트 Zybo 프리셋과 대조함
- DDR 보드 지연값은 ALINX 가 안 알려줘서 Zybo 값 + DDR 자동 보정 3가지 켬
- 실물 모듈 · 어댑터 · 서보로는 아직 안 돌림 (USB3320 의 VBUS 를 PHY 가 켜는지 점퍼가 주는지 모름 → 두 경우 다 되게 함)

## 6. 바로잡은 것

- 예전 `fpga/tpu/vivado/build.tcl` 의 DDR 부품 `MT41K256M16` 은 **AC7020C(1GB)용**이다. AC7010C 는 매뉴얼 6.2 대로 `H5TQ2G63FFR` = **MT41J128M16 ×2 = 512MB**. 여기 `ps7_ac7010c.tcl` 은 맞는 값으로 했다
- 로봇판 v3.5 xdc 의 `create_clock ... cam_pclk` 는 v3.5 에 없는 포트라 뺐다
