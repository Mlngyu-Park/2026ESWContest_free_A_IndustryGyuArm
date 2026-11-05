# KYLE · HOTEL — 이 폴더에서 일할 때 지킬 것

- **KYLE** = 만능기판 손납땜 회로를 적는 언어 (`.txt`, 첫 줄 `#KYLE 1`). 명세는 `KYLE.md`.
- **HOTEL** = 그 언어를 검증·배치·배선·3D·에뮬레이션하는 프로그램 전체. 안내서는 `HOTEL.md` (파일 목록·명령·실측·남은 것).
- **파일이 진실**: 배치·배선·버스·시험(sim) 전부 KYLE 텍스트에 있어야 한다. 뷰어/배치기가 만든 것도 반드시 파일로 되돌아간다.
- **AI 는 연결(pin)과 기대값(expect/sim)만 적는다.** 전압·시간·규칙 판정은 sol.py 가 chips.txt 의 데이터시트 숫자로 계산한다. 숫자를 지어내지 말고, 모르면 "계산값" 이라고 말한다.
- 뷰어를 고치면 **스크린샷을 렌더링해 직접 본다** (검사 통과 ≠ 화면 맞음). 시험: `test_viewer.py`, `test_place.py`, `test_sim.py`, `test_panel.py` — 고치면 돌린다.
- 시험은 `fixture_2pe.txt` 를 쓴다. `pe2.txt` 는 사용자의 작업 파일이라 기준이 아니다.
- 배선 도구(route_eval, test_viewer, test_panel)는 playwright+chromium 이 필요하다. DGX 에서는 `~/project/kyle_venv/bin/python3` 를 쓴다.
- 말투: 존댓말, 용어는 풀어서. 확인 안 된 것은 확인 안 됐다고 쓴다.

빠른 시작
```
python3 sol.py pe2.txt                 # 검증 (R1~R30)
python3 serve.py pe2.txt               # 뷰어 (브라우저 http://127.0.0.1:8765)
python3 simulate.py pe2.txt --run      # 파일 안의 sim 시나리오 실행
python3 place.py pe2.txt --out out.txt --board 54 33 --layers 2 --search 4 --route
python3 route_eval.py out.txt --write  # 배선을 파일에 써 넣기
python3 space.py --pes 1 --board 80 50 --layers 1   # PE 하나에 필요한 기판 실측
```

TPU 관련 문서 (`tpu_docs/`)
- `tpu_docs/PROJECT.md` — 자작 LLM(RACHEL) 과 자작 TPU 전체 정리
- `tpu_docs/m68k_array/TPU.md` — 시스톨릭 배열을 IC 로 만드는 구상, `HARDWARE.md` — 68000 병렬 컴퓨터 하드웨어
- `tpu_docs/tpu_pe/` — PE 2개 시제품 (README·배선표 WIRING·구매 BUY, 옛 배선목록·레이아웃·곱셈표 ROM 생성기)
