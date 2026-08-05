#!/usr/bin/env python3
"""관절 한계 측정값 (raw). 2026-07-24 measure_limits.py 로 손으로 측정 (재측정, 깔끔).
20260724 로봇 (6축). 전부 0~4095 범위 안 (경계 문제 없음)."""

JOINT_LIMITS_RAW = {
    "j1": (415,  3753, 293.4),
    "j2": (493,  2629, 187.7),
    "j3": (694,  3232, 223.1),
    "j4": (819,  3387, 225.7),
    "j5": (1599, 3524, 169.2),
    "j6": (1970, 3525, 136.7),   # 그리퍼
}
MOTOR_ID = {"j1":0,"j2":1,"j3":2,"j4":3,"j5":4,"j6":5}

if __name__=='__main__':
    print("측정된 관절 한계 (raw), 2026-07-24:")
    for j,(mn,mx,sp) in JOINT_LIMITS_RAW.items():
        print(f"  {j} (ID{MOTOR_ID[j]}): raw {mn}~{mx}  범위 {sp:.1f}deg")
