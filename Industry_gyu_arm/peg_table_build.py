#!/usr/bin/env python3
"""목심 도달표 미리 계산기 (오래 걸리는 계산을 앱과 분리)

각 목심에 대해 z 를 1mm 씩 훑어 '도달 가능한 높이 구간들' 을 구해 파일로
저장한다. 앱(arm_studio_web.py)은 시작할 때 이 파일이 있으면 즉시 불러
쓰므로, 높이 슬라이더를 움직여도 계산 없이 바로 반응한다.

사용:
    conda run --no-capture-output -n gyu_arm python peg_table_build.py
    conda run --no-capture-output -n gyu_arm python peg_table_build.py --max-h 300

저장 위치: <base URDF 패키지>/peg_table.json
    로봇 위치(BASE_OFF), URDF, 플레이트 배치가 바뀌면 자동으로 무효가 되어
    앱이 파일을 무시한다. 그때는 이 스크립트를 다시 돌리면 된다.

주의: 앱이 켜져 있어도 상관없다(파일만 쓴다). 다만 계산 중에는 CPU 를
      많이 쓰므로, 실물 로봇을 돌리는 중이라면 끝난 뒤에 실행하는 게 좋다.
"""
import sys, os, json, time, argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--max-h', type=int, default=None,
                    help='표에 담을 최대 높이 mm (기본: 앱의 PEG_H_MAX)')
    ap.add_argument('--out', default=None, help='저장 경로 직접 지정')
    args = ap.parse_args()

    print('앱 모듈을 불러오는 중... (URDF/메쉬 로드로 20초쯤 걸립니다)')
    import arm_studio_web as m

    if args.max_h:
        m.PEG_H_MAX = args.max_h
    out = args.out or os.path.join(m.URDF_PKG_DIR, 'peg_table.json')

    n_peg = len(m.peg_names())
    print(f"\n로봇 위치 : x={m.BASE_OFF[0]*1000:+.0f} y={m.BASE_OFF[1]*1000:+.0f} mm")
    print(f"플레이트  : {sorted(m.PLATES.keys())}")
    print(f"목심      : {n_peg}개  (플레이트당 10x10, 간격 {m.PEG_PITCH*1000:.0f}mm)")
    print(f"높이 범위 : 0 ~ {m.PEG_H_MAX}mm, 1mm 간격")
    print(f"워커      : {m._worker_count()}개")
    print(f"총 IK     : 약 {n_peg*(m.PEG_H_MAX+1):,}회\n")

    t0 = time.time()
    span, el = m.build_peg_table()
    el = time.time() - t0

    multi = {nm: sp for nm, sp in span.items() if len(sp) > 1}
    data = {
        'key': list(m.peg_table_key()[:2]) + [m.peg_table_key()[2],
                                              [list(t) for t in m.peg_table_key()[3]]],
        'max_h': m.PEG_H_MAX,
        'pitch': m.PEG_PITCH,
        'built': time.strftime('%Y-%m-%d %H:%M:%S'),
        'span': {nm: [list(s) for s in sp] for nm, sp in span.items()},
    }
    with open(out, 'w') as f:
        json.dump(data, f)

    print(f"\n완료 {el:.0f}초")
    print(f"  도달 가능      : {len(span)}/{n_peg}개")
    print(f"  구간이 끊긴 것 : {len(multi)}개"
          + (f"  {list(multi)[:4]}" if multi else ""))
    print(f"  저장           : {out}  ({os.path.getsize(out)/1024:.0f}KB)")
    print("\n앱을 재시작하면 이 표를 바로 불러 씁니다.")

if __name__ == '__main__':
    main()
