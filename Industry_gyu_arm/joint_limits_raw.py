#!/usr/bin/env python3
"""
관절 한계 측정값 (raw). 2026-07-20 measure_limits.py 로 손으로 돌려 측정.
- 값은 Dynamixel Present Position raw (0~4095, 단일회전 Position 모드 기준).
- 주의: raw 절대값은 재연결/모드변경 때 원점이 달라질 수 있음.
  신뢰할 것은 각 관절의 '가동 범위 폭(span_deg)'.
- rad 한계로 변환하려면 영점(0자세 raw)이 정해져야 함:
    lower_rad = (min_raw - ZERO_RAW) * pi / 2048
    upper_rad = (max_raw - ZERO_RAW) * pi / 2048
"""

# {joint: (min_raw, max_raw, span_deg)}
JOINT_LIMITS_RAW = {
    "j1": (1300, 3606, 202.7),
    "j2": (421,  2806, 209.6),
    "j3": (692,  3104, 212.0),
    "j4": (1215, 3398, 191.9),
    "j5": (1522, 3583, 181.1),
    "j6": (375,  3768, 298.2),   # 그리퍼
}

MOTOR_ID = {"j1": 0, "j2": 1, "j3": 2, "j4": 3, "j5": 4, "j6": 5}

# 영점(0자세) raw — 2026-07-20 사용자가 기본자세 잡고 캡처.
ZERO_RAW = {"j1": 2406, "j2": 1441, "j3": 1430, "j4": 2319, "j5": 2532, "j6": 538}

# 위 영점 기준으로 변환한 rad 한계 (j1~j5 회전, j6은 그리퍼라 별도 처리 필요).
JOINT_LIMITS_RAD = {
    "j1": (-1.697, 1.841),
    "j2": (-1.565, 2.094),
    "j3": (-1.132, 2.568),
    "j4": (-1.694, 1.655),
    "j5": (-1.549, 1.612),
    "j6": (-0.250, 4.955),   # 그리퍼: raw 375~3768, URDF에선 prismatic 별도 매핑
}


def to_rad_limits(zero_raw):
    """영점 dict {joint: raw} 를 주면 rad 한계 dict 반환."""
    import math
    out = {}
    for j, (mn, mx, _) in JOINT_LIMITS_RAW.items():
        z = zero_raw[j]
        lo = (mn - z) * math.pi / 2048.0
        hi = (mx - z) * math.pi / 2048.0
        out[j] = (round(lo, 4), round(hi, 4))
    return out


if __name__ == "__main__":
    print("측정된 관절 한계 (raw):")
    for j, (mn, mx, span) in JOINT_LIMITS_RAW.items():
        print(f"  {j} (ID{MOTOR_ID[j]}): raw {mn}~{mx}  범위 {span:.1f}deg")
