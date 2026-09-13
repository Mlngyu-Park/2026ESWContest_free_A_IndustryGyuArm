말로 명령하는 블록 코딩 로봇팔 제어 · 디지털 트윈 - 인규암

영상 링크 (https://www.youtube.com/watch?v=rnEmlPO3QYE)

사용 디바이스
- SO-ARM101 로봇팔 (Feetech STS3215 서보 6축)
- NVIDIA DGX Spark (LLM/STT 서버)
- 제어 PC (Ubuntu 22.04 / WSL2)

### 공장 꾸미기
- 설비 STL 드래그 앤 드롭 배치 — 실물 모양이 커서를 따라오고, 핀 2개를 목심에 클릭해 자리·회전 확정
- 개체별 이름·색 지정, 바닥 플레이트 편집, 로봇 위치 이동

### 블록 코딩
- 스크래치식 팔레트: **이동 / 잡기 / 놓기 / 그리퍼 / 하강 / 상승**
- 이동 블록은 실행 시점에 대상 위치를 재계산 — 물체가 중간에 옮겨져도 따라간다
- Ctrl+Z 실행취소, 전체 블록 3D 미리보기

### 음성 명령
- 마이크 → Whisper(한국어 STT, 실시간 자막) → **파인튜닝 LLM** → 블록 자동 생성
- "빨간 도킹의 초록 펜을 노란 데 놓고 다시 주황으로 옮겨" 같은 2단계 이송까지 실측 2~4초
- 서버가 없으면 규칙 파서로 자동 전환 (오프라인 동작)

### 실물 실행
- 시뮬과 실물이 같은 파이프라인 공유 — 3D는 이론 자세, 실물에만 보정값 적용
- 그리퍼 파지 감지: 닫는 중 위치가 멈추면 잡은 것으로 판정, 과부하 방지
- 정지 시 즉시 토크 해제, 관절 한계 EEPROM 기록

## 🔬 핵심 기술

### 기구학 직접 구현
- URDF 파싱 → 관절 체인 구성, 순기구학은 동차변환 행렬 곱
- 역기구학은 시드 기반 반복 최적화로 직접 구현 (목적별 3종: 일반 6축 / 툴 중심 / 수평 구속)
- **수평 구속**: j2·j3·j4가 같은 회전축을 공유하므로 세 관절각의 합이 엔드이펙터
  기울기를 결정한다. 이 합을 경로 전 구간 상수로 유지하고 j5를 잠가,
  물건을 든 채 이동해도 기울지 않는다. 미세조정 보정도 같은 구속을 지킨다.
- 작업공간 도달표 사전 계산, 로봇 상대좌표 저장 — 로봇·설비를 옮겨도 재계산 0초
- fcl 충돌검사, trimesh 메쉬 처리

### LLM 파인튜닝
- **Gemma 4 E4B** 를 자체 제작 한국어 명령 데이터셋 **1,568쌍**으로 LoRA 파인튜닝
  (데이터셋 생성: `make_llm_data.py`, 모델 가중치는 용량 관계로 미포함)
- 출력을 블록 JSON 한 줄로 제한 (평균 58자) → 작은 모델로도 1~4초 응답
- LLM 출력 검증 계층: id 정규화, 잡기 앞 이동 블록 보장, 장면에 없는 대상은
  블록 폐기 후 되물음 — **LLM은 블록만 고르고, 관절값·경로는 전부 기구학 계산**

### 제어 정밀도
| 게인 | 정지 오차 |
|---|---|
| 공장 기본 (Kp32/Kd32/Ki0) | 1.58° |
| 실측 튜닝 (Kp64/Kd64/Ki4) | **0.35° · 진동 0** |

도달 허용치 6 raw ≈ 0.53° ≈ 엔드이펙터 3mm

## 🛠 시스템 요구사항

### 하드웨어
- SO-ARM101 (STS3215 × 6) + USB 시리얼 어댑터
- 마이크 (음성 명령용, 선택)
- LLM/STT 서버 (선택 — 없어도 규칙 파서로 동작)

### 소프트웨어
- Python 3.10+ / Conda
- Chrome 또는 Firefox

## 📦 설치 및 실행

### 1. 환경 설정
```bash
git clone https://github.com/Mlngyu-Park/2026ESWContest_free_A_IndustryGyuArm.git
cd 2026ESWContest_free_A_IndustryGyuArm
conda create -n gyu_arm python=3.10
conda activate gyu_arm
pip install fastapi uvicorn numpy trimesh python-fcl ikpy pyserial
```

### 2. (WSL 사용 시) USB 연결
```powershell
# Windows PowerShell
usbipd attach --wsl --busid <busid>
```

### 3. 실행
```bash
Industry_gyu_arm/start_pro.sh
# 브라우저에서 http://localhost:7900  (3D 뷰어: 7870)
```

LLM/STT 서버 터널 주소는 `Industry_gyu_arm/spark_tunnel.txt` 에 넣는다.
없으면 규칙 파서로 동작한다.

## 📁 프로젝트 구조

```
2026ESWContest_free_A_IndustryGyuArm/
├── Industry_gyu_arm/
│   ├── arm_studio_web.py      # 코어: 기구학·경로·모터·LLM 연동 (약 1만 줄)
│   ├── arm_studio_pro.py      # FastAPI 서버 (/api/call 라우팅)
│   ├── pro_ui/                # 블록 코딩 UI (바닐라 JS + three.js)
│   ├── make_llm_data.py       # LLM 학습 데이터셋 생성 (1,568쌍)
│   ├── start_pro.sh           # 실행 스크립트
│   ├── sts_zero.py 등         # 모터 캘리브레이션 데이터
│   └── dock_mate_fixed.py 등  # 실측 결합 치수
├── urdf/                      # 로봇 모델 (SO-ARM · Dynamixel · UR15)
├── stl/                       # 설비·물체 메쉬
└── docs/                      # 문서·이미지
```

## 🎮 사용 방법

1. **① 공장 꾸미기** 탭에서 설비를 드래그해 배치
2. **② 시뮬레이션** 탭에서 블록을 조립하거나, 마이크 버튼을 누르고 말하기
3. ▶ 미리보기로 3D 검증
4. **③ 연동** 탭에서 실물 로봇 선택 후 실행

## 🔧 기술 스택

### 제어/기구학
- **numpy**: 동차변환·IK 수치 최적화
- **trimesh + fcl**: 메쉬 처리·충돌검사
- **pyserial**: STS3215 서보 통신

### AI
- **llama.cpp**: Gemma 4 E4B 파인튜닝 모델 서빙
- **whisper.cpp**: 한국어 STT (large-v3-turbo)

### UI/서버
- **FastAPI**: API 서버
- **three.js**: 3D 디지털 트윈 뷰어
- **바닐라 JS**: 블록 코딩 UI

## ❗ 문제 해결

### USB 인식 안 됨 (WSL)
```powershell
usbipd list
usbipd attach --wsl --busid <busid>
```

### 음성 인식이 안 됨
- LLM/STT 서버 연결 확인 — 끊겨 있으면 규칙 파서로 동작하며 기본 명령만 인식
- 브라우저 마이크 권한 허용 확인

### 포트 충돌
```bash
# start_pro.sh 가 자동 정리하지만, 수동으로는:
pkill -f arm_studio_pro.py
```

## 👥 만든 사람

팀장 : 박민규 (제작총괄, 알고리즘, 매커니즘, CAD, llm 파인튜닝, 코딩)
부팀장 : 허준행 (제작 보조, CAD 보조, 아이디어)
팀원 1 : 노연우 (조립)
팀원 2 : 이윤채 (서류작업)
팀원 3 : 송민성 (서류 보조)
