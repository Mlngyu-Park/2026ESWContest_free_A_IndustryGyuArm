#!/bin/bash
# RACHEL 정확도 진단 — 정답(C++)과 dusk 를 나란히 본다
#   ./diag.sh          기본 문장
#   ./diag.sh rank     첫 글자 순위만 (제일 빠름)
cd "$(dirname "$0")"
W=../rachel_cpp/rachel.bin
P='보라=d2 빨강=d10 주황=d11 | 펜=초록@d2
힌트 d2 d10
보라색 펜을 빨간 도킹으로 옮겨
>'

if [ "$1" = "rank" ]; then
    DUSK_RANK='{' timeout 300 ./dusk --weights $W --ask "$P" 2>&1 | grep 순위
    exit 0
fi

echo "=== 정답 (C++) ==="
../rachel_cpp/rachel $W "$P" 2>/dev/null

echo ""
echo "=== dusk (정수) ==="
timeout 300 ./dusk --weights $W --ask "$P" 2>&1 | grep '^출력' | cut -c1-90

echo ""
echo "=== 첫 글자 순위 (1위가 목표) ==="
DUSK_RANK='{' timeout 300 ./dusk --weights $W --ask "$P" 2>&1 | grep 순위

echo ""
echo "=== 층별 잔차 (정답은 rms 2.48 -> 6.72 로 자란다) ==="
DUSK_DBG2=1 timeout 300 ./dusk --weights $W --ask "$P" 2>&1 | grep '^  L' | head -8

echo ""
echo "=== 어텐션 (균등하면 문맥을 못 읽는 것) ==="
DUSK_ATT=1 timeout 300 ./dusk --weights $W --ask "$P" 2>&1 | grep 'att pos' | head -4
