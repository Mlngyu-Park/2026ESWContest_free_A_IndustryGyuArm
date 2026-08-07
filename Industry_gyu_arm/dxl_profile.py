# ===== Dynamixel(옛 로봇, Industry_gyu_arm 20260723) 프로필 =====
# 출처: 2026-07-24 실측 기록 (메모리/이전 세션)
#  - XC430(ID0,3) / XM430(ID1,2,4), Protocol 2.0, baud 57600
#  - j4<->j5 배선 교차: RViz j4 = 모터 ID4, RViz j5 = 모터 ID3
#  - j1(ID0) 만 방향 반대

PORT='/dev/ttyUSB0'          # U2D2
BAUD=57600
PROTOCOL=2.0

# 조인트 인덱스(j1~j6) -> 모터 ID.  주의: j4<->j5 교차!
J2ID=[0, 1, 2, 4, 3, 5]

# 방향 부호 (ID 기준)
DIR={0:-1, 1:1, 2:1, 3:1, 4:1, 5:1}

# 관절 한계 raw (ID 기준, 2026-07-24 재측정)
LIMIT_RAW={0:(380,2571), 1:(1479,3555), 2:(1334,3382),
           3:(1271,3153), 4:(1599,3524), 5:(1970,3525)}

# 영점 (별도 파일 - 재캡처 시 그 파일만 갱신)
from dxl_zero import ZERO_RAW, VERIFIED as ZERO_VERIFIED

# 그리퍼(ID5) 열림/닫힘 raw - 미측정. 측정 전까지 그리퍼 자동제어 안 함
GRIP_OPEN=None
GRIP_CLOSE=None
