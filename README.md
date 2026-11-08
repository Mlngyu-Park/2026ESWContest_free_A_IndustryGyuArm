# 인규암 (Industry Gyu Arm)

**자연어 음성 제어 · 디지털 트윈 검증 기반 피지컬 AI 로봇팔 플랫폼**

제24회 임베디드SW경진대회 [자유공모]

## 🎬 영상

| [![임베디드SW경진대회 시연 영상](https://img.youtube.com/vi/rnEmlPO3QYE/hqdefault.jpg)](https://www.youtube.com/watch?v=rnEmlPO3QYE) | [![RACHEL 기반 자연어 음성 로봇팔 제어](https://img.youtube.com/vi/7IwZH8eliuM/hqdefault.jpg)](https://www.youtube.com/watch?v=7IwZH8eliuM) |
|---|---|
| **임베디드SW경진대회 시연 영상** (2026-09) | **한이음 드림업 — 자체 개발 AI 'RACHEL' 기반 자연어 음성 로봇팔 제어 · 디지털 트윈** (2026-09) |

두 영상은 9월에 찍었다. 그 뒤에 두 팔 제어, 더블 레이첼, DUSK_BOOM TPU 가 더해졌다.

---

## 왜 만들었나

**로봇은 늘어나는데, 다룰 사람이 없다.**
- 산업통상자원부 목표: 2030년까지 로봇 100만 대 이상 보급, 그중 제조·물류 현장에 68만 대. 국내 로봇 시장 5.6조 원 → 20조 원 이상.
- 2024년 로봇산업 실태조사: 로봇산업 인력은 1년간 2.4% 늘었지만 2031년 인력 수요는 약 5만 명. 로봇 분야 인력부족률 35% (한국경제연구원, 2020). 제조업용 로봇 분야 인력은 오히려 0.1% 줄었다.
- 현장에서 로봇이 실제로 하는 일은 **이송·적재 같은 간단한 반복 동작**이다. 다만 물건의 위치와 작업 순서를 판단해야 해서, 작업 과정 전체를 이해하는 AI 가 필요하다.

**그런데 AI 를 로봇에서 돌리는 비용이 오르고 있다.**
- 2026년 로봇용 추론 장치 가격은 메모리 가격 여파로 최대 101% 올랐다.
- 대표적인 로봇 AI 모델 OpenVLA 는 실행에만 GPU 메모리 15GB 이상이 필요하다 → 중소 현장은 도입이 어렵다.

## 목표 세 가지

| 목표 | 해결 |
|---|---|
| 비전공자도 쉽게 쓸 수 있을 것 | **자연어 제어** — 말로 시키면 블록 코딩으로 바뀐다 |
| 낮은 컴퓨팅 파워로 AI 를 돌릴 것 | **로컬 AI + TPU** — 25MB 자작 모델 [RACHEL](RACHEL.md), 직접 설계한 FPGA TPU [DUSK_BOOM](DUSK_BOOM.md) |
| AI 에 전적으로 의존하지 않을 것 | **디지털 트윈** — AI 는 블록까지만. 움직임은 계산하고, 시뮬레이션으로 먼저 검증 → [ARM STUDIO](ARM_STUDIO.md) |

## 작동 파이프라인

| 단계 | 하는 일 | 어디서 |
|---|---|---|
| 1. 말하기 | 마이크에 대고 한국어로 명령 | |
| 2. 자연어 인식 | 음성 → 글자 (STT), 글자 → 블록 (**RACHEL**) | LattePanda Mu CPU + DUSK_BOOM TPU |
| 3. 디지털 트윈 | 블록 → 경로 계산 → 3D 시뮬레이션으로 검증 (경로 · 충돌 · 물병 규칙) | ARM STUDIO |
| 4. 실물 실행 | 검증된 동작만 두 팔 로봇으로 | ARM STUDIO → 모터 |

예: "펜을 잡고 파란색 도킹스테이션으로 옮기고, 다음에 빨간색으로 옮겨"
→ 잡기 펜 → 이동 파랑 → 놓기 → 잡기 펜 → 이동 빨강 → 놓기 → 3D 확인 → 실물 실행

---

## 핵심 기술

### 1. [RACHEL — 말을 블록으로 바꾸는 25MB 자체 AI](RACHEL.md)  ★ 메인

- 사람의 언어(자연어)와 로봇의 언어(모터 제어) 사이의 **중간 다리 = 블록**.
- PyTorch 로 직접 짠 글자 단위 트랜스포머. 파라미터 2,560만 개, int8 가중치 25.6MB.
- 학습 데이터는 규칙으로 합성 (사람이 라벨링한 데이터 0줄), DGX Spark GPU 1장으로 학습.
- 출력은 계산으로 9가지를 검사한 뒤에만 블록이 된다. 의심스러우면 실행하지 않는다.
- 사용자의 실제 음성 명령 87문장 **87/87**. 같은 200문장에서 Gemma 4 E4B 39.0% vs RACHEL 98.5%.

### 2. [DUSK_BOOM — 직접 설계한 행렬곱 TPU](DUSK_BOOM.md)

- Zynq XC7Z010 FPGA 에 Verilog 로 설계한 TPU. 곱셈기 64개(8×8), int8 × int16.
- 최대 2.5W (N305 CPU 최대 21W). 더블 레이첼 한 문장 2.66초, 노트북 CPU 와 60/60 같은 답.
- 보드 리눅스 부팅, HDMI 출력 회로, TPU 시연 화면까지 직접.

### 3. [ARM STUDIO — 블록 코딩 · 디지털 트윈 · 실물 제어](ARM_STUDIO.md)

- 블록 → 도달표 → 수직선 정렬 → 최단 경로 → 검증(물병 규칙) → 3D 시뮬레이션 → 실물.
- 기구학 직접 구현, 두 팔 제어, 사람이 맞춘 자리 보정을 그대로 사용.
- 블록은 물건의 상태와도, 로봇의 종류와도 상관없이 쓸 수 있다.

---

## 하드웨어 구성

```
LattePanda Mu — 메인 컴퓨터 (N305 16GB + 자체 설계 캐리어보드)
├─ USB ◀─ 마이크
├─ HDMI ─▶ 모니터 (ARM STUDIO 화면)
├─ USB 시리얼 ─▶ DUSK_BOOM (Zynq XC7Z010 · 512MB · TPU 8×8)
│     └─ HDMI ─▶ 모니터 (TPU 계산 화면)
└─ USB ─▶ 모터 드라이버
      ├─ TTL ─▶ A팔 (STS3215 · 12V 5A)
      └─ TTL ─▶ B팔 (STS3215 · 12V 5A)
```

## 개발 환경

| 구분 | | 내용 |
|---|---|---|
| S/W | OS | Ubuntu 22.04 (실행 보드), Windows 11 + WSL2 (개발 노트북) |
| | 개발 환경 | VS Code, Python 3.10, ROS 2 Humble |
| | 개발 도구 | PyTorch, FastAPI, three.js, RViz, trimesh, python-fcl, Vivado |
| | AI 모델 | 음성 인식 (Whisper large-v3-turbo · Moonshine), 더블 레이첼 (자체 개발 블록 AI, int8 25.6MB) |
| | 언어 | Python, C++, C, JavaScript, Verilog |
| H/W | 실행 장치 | LattePanda Mu N305 16GB + 자체 설계 캐리어보드 |
| | 학습 장치 | NVIDIA DGX Spark (학습 전용) |
| | AI 가속기 | DUSK_BOOM (Zynq XC7Z010 + 자체 설계 행렬곱 TPU) |
| | 로봇 팔 | SO-ARM101 개선판 (Feetech STS3215) × 2, OpenManipulator |
| | 기타 | USB 마이크, 아두이노 비상정지 버튼, 도킹 스테이션 · 바닥판 (3D 프린트) |
| 관리 | 형상 관리 | GitHub |
| | 의사소통 | Slack, Notion, Zoom |

---

## 저장소 구조

```
├── README.md            ← 지금 이 문서
├── RACHEL.md            ← 자작 AI (메인)
├── DUSK_BOOM.md         ← FPGA TPU
├── ARM_STUDIO.md        ← 블록 코딩 · 디지털 트윈 · 실물 제어
├── Industry_gyu_arm/
│   ├── nano_double/     # 더블 레이첼: 데이터 생성 · 학습 · 검사 · 채점
│   ├── nano_norm/       # 말투 엔진 · 글자 사전 · 모델 구조 (PyTorch)
│   ├── nano_llm/        # 한 팔 RACHEL (v1~v9)
│   ├── rachel_cpp/      # C++ 추론 엔진
│   ├── dusk/            # 정수 전용 추론 엔진 DUSK
│   ├── arm_studio_web.py, arm_studio_pro.py, pro_ui/   # ARM STUDIO
│   └── start_pro.sh
├── dusk_boom_fpga_v35/
│   ├── rtl/             # TPU · HDMI 회로 (Verilog)
│   ├── sim/ vivado/ constraints/
│   ├── tpu/sw/          # 보드 TPU 판 RACHEL
│   ├── tpu_show/        # TPU 시연 화면
│   ├── linux/           # 보드 리눅스
│   └── mu/              # LattePanda Mu 실행 스크립트
├── urdf/                # 로봇 모델 (SO-ARM STS3215 · OpenManipulator Dynamixel)
└── stl/                 # 설비 · 물체 메쉬
```

모델 가중치, 로봇별 보정 데이터(`calib.json`)는 저장소에 넣지 않았다. 만드는 방법은 각 문서에 있다.

## 빠른 실행 (노트북)

```bash
git clone https://github.com/Mlngyu-Park/2026ESWContest_free_A_IndustryGyuArm.git
cd 2026ESWContest_free_A_IndustryGyuArm
conda create -n gyu_arm python=3.10 && conda activate gyu_arm
pip install fastapi uvicorn numpy trimesh python-fcl ikpy pyserial
Industry_gyu_arm/start_pro.sh          # 브라우저에서 http://localhost:7900
```

WSL 에서 로봇 USB 연결, LattePanda Mu 실행은 [ARM_STUDIO.md](ARM_STUDIO.md) 8장.

---

## 👥 만든 사람

팀장 : 박민규 (제작총괄, 알고리즘, 매커니즘, CAD, llm 파인튜닝, 코딩)
부팀장 : 허준행 (제작 보조, CAD 보조, 아이디어)
팀원 1 : 노연우 (조립)
팀원 2 : 이윤채 (서류작업)
팀원 3 : 송민성 (서류 보조)
