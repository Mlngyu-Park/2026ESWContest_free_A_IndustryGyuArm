#!/bin/bash
# DUSK 빌드 — 파이썬 없이 도는 68000 세계
#   ./build.sh          전부 빌드
#   ./build.sh test     커널 검증만
cd "$(dirname "$0")"
set -e
CC=${CC:-gcc}
FLAGS="-O2 -Wall"

echo "▸ 정수 커널 검증 빌드"
$CC $FLAGS -o test_node test_node.c dusk_node.c -lm
echo "▸ 커널 검증 실행"
./test_node

if [ "$1" = "test" ]; then exit 0; fi

echo ""
echo "▸ DUSK CPU+GPU 구조 빌드"
$CC $FLAGS -c dusk.c -o dusk.o
echo "  완료 (dusk.o)"
