#!/usr/bin/env python3
"""목심 도달표 미리 계산기 (오래 걸리는 계산을 앱과 분리)

로봇 기준 상대좌표 격자 (10+20i, 10+20j)mm 의 각 칸에 대해 z 를 1mm 씩 훑어
'도달 가능한 높이 구간들' 을 구해 파일로 저장한다. 도달 가능성은 로봇 기준
상대 위치의 성질이라, 로봇을 어디로 옮기든(90도 회전 포함) 이 표는 그대로
유효하다 -> URDF 가 바뀌지 않는 한 다시 돌릴 일이 없다.

사용:
    conda run --no-capture-output -n gyu_arm python peg_table_build.py
    conda run --no-capture-output -n gyu_arm python peg_table_build.py --max-h 300

저장 위치: <base URDF 패키지>/peg_table.json

주의: 앱이 켜져 있어도 상관없다(파일만 쓴다). 다만 계산 중에는 CPU 를
      많이 쓰므로, 실물 로봇을 돌리는 중이라면 끝난 뒤에 실행하는 게 좋다.
"""
import sys, os, time, argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-h', type=int, default=None,
                    help='표에 담을 최대 높이 mm (기본: 앱의 PEG_H_MAX)')
    args = ap.parse_args()

    print('앱 모듈을 불러오는 중... (URDF/메쉬 로드로 20초쯤 걸립니다)')
    import arm_studio_web as m

    if args.max_h:
        m.PEG_H_MAX = args.max_h
    out = os.path.join(m.URDF_PKG_DIR, 'peg_table.json')

    cells = m._rel_cells()
    print(f"\nURDF      : {m.BASE_URDF}")
    print(f"상대 격자 : {len(cells)}칸  (10+20i, 10+20j)mm, 도달 반경 안")
    print(f"높이 범위 : 0 ~ {m.PEG_H_MAX}mm, 1mm 간격 (높이별 링 선별로 IK 절약)")
    print(f"워커      : {m._worker_count()}개\n")

    span, el = m.build_peg_table()      # 계산 + 저장까지 한다
    multi = {k: sp for k, sp in span.items() if len(sp) > 1}
    print(f"\n완료 {el:.0f}초")
    print(f"  도달 가능 칸   : {len(span)}/{len(cells)}")
    print(f"  구간이 끊긴 칸 : {len(multi)}개"
          + (f"  {list(multi)[:4]}" if multi else ""))
    print(f"  저장           : {out}  ({os.path.getsize(out)/1024:.0f}KB)")
    print("\n앱을 재시작하면 이 표를 바로 불러 씁니다. 로봇을 옮겨도 재계산 없음.")

if __name__ == '__main__':
    main()
