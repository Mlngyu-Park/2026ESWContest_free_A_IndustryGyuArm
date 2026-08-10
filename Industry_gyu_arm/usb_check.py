#!/usr/bin/env python3
"""연결된 로봇 어댑터 확인 (통신은 하지 않음 - 로봇을 움직이지 않는다).

포트 번호(/dev/ttyACM0 등)는 뽑았다 꽂거나 usbipd 를 다시 attach 하면
0 -> 1 -> 2 로 밀린다. 대신 /dev/serial/by-id/ 에는 칩 종류와 시리얼번호가
박힌 고정 경로가 있어, 그걸로 어느 로봇인지 확실히 구별할 수 있다.

사용: python usb_check.py
"""
import os, glob

BYID='/dev/serial/by-id'

def main():
    print("=== 연결된 시리얼 어댑터 ===")
    items=sorted(glob.glob(os.path.join(BYID,'*')))
    if not items:
        print("  (없음) — Windows PowerShell 에서 usbipd attach 를 먼저 하세요:")
        print("     usbipd list                       # busid 확인")
        print("     usbipd attach --wsl --busid <N-N>")
        return
    for p in items:
        n=os.path.basename(p); low=n.lower()
        real=os.path.realpath(p)
        if 'ftdi' in low or '0403' in low:
            who='Dynamixel (U2D2, FTDI 칩)'
        elif '1a86' in low or 'ch343' in low or 'ch340' in low:
            who='SO-ARM (STS3215, CH343 칩)'
        else:
            who='알 수 없는 어댑터'
        print(f"\n  {who}")
        print(f"    고정 경로: {p}")
        print(f"    현재 번호: {real}")
    print("\n=== 앱이 사용할 포트 ===")
    import sys
    sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
    try:
        # 앱 전체를 띄우지 않고 스캔 로직만 흉내 (URDF 로드 없이 빠르게)
        sts=dxl=None
        for p in items:
            low=os.path.basename(p).lower()
            if ('ftdi' in low or '0403' in low) and not dxl: dxl=p
            elif ('1a86' in low or 'ch343' in low) and not sts: sts=p
        print(f"  SO-ARM    : {sts or '없음'}")
        print(f"  Dynamixel : {dxl or '없음'}")
    except Exception as e:
        print("  확인 실패:",e)
    print("\n번호가 밀려도 앱은 위 고정 경로로 찾으므로 그대로 동작합니다.")

if __name__=='__main__':
    main()
