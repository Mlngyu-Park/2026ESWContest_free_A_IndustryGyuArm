#!/bin/bash
# 도킹 메이트(펜이 도킹에 꽂히는 자리) 정하기 - 1회용 도구
#   ./pick_mate.sh   -> 브라우저에서 도킹 소켓 원 + 펜 원 클릭 -> 저장
# 저장되면 dock_mate_fixed.py 가 생기고, ARM STUDIO 재시작 시 그 값만 쓴다.
cd "$(dirname "$0")"
exec conda run --no-capture-output -n gyu_arm python pick_dock_mate.py "$@"
