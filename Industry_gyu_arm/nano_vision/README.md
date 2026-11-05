# nano_vision — 카메라 비전 AI (조사 단계)

**아직 만들지 않았다. 조사만 했다.**

- [RESEARCH.md](RESEARCH.md) — **조사 결과. 이걸 읽으면 된다.**

## 결론

CPU only 60fps 는 **된다.** 단 10MB 를 다 쓰면 41fps 이고,
6MB 로 줄이고 2프레임에 한 번만 신경망을 돌리면 **80fps** 다.

## 실측 스크립트 (재현용)

```bash
python3 bench_vision.py     # 모델 크기별 / 해상도별 / 스레드별
python3 bench_int8.py       # int8 양자화 효과 (핵심: 3.18배)
python3 bench_pipeline.py   # 신경망 밖 비용 (핵심: 13ms 샘)
python3 bench_tracking.py   # 추적으로 메우기 (핵심: 40 -> 80fps)
```

무작위 가중치로 **속도만** 잰다. 정확도는 측정하지 않는다.

## RACHEL 과 반대인 두 가지 (주의)

| | RACHEL | 비전 |
|---|---|---|
| 양자화 | W8A32 (가중치만) | **W8A8 (전면)** — VNNI 가속 |
| 스레드 | OpenMP 뺌 (4배 빨라짐) | **넣음** (8스레드 4.1배) |
