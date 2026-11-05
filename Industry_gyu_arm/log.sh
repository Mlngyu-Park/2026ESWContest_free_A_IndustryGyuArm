#!/bin/bash
# 서버 로그 보기
#   ./log.sh          최근 40줄
#   ./log.sh -f       실시간 따라가기
#   ./log.sh 100      최근 100줄
L=/tmp/arm_studio.log
[ -f "$L" ] || { echo "로그 없음 — start_pro.sh 로 서버를 시작하세요"; exit 1; }
case "$1" in
  -f) exec tail -f "$L" ;;
  '') tail -40 "$L" ;;
  *)  [ "$1" -gt 0 ] 2>/dev/null && tail -"$1" "$L" || tail -40 "$L" ;;
esac
