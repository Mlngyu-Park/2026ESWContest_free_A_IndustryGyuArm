#!/bin/bash
# start_spot.sh — 뮤: ARM STUDIO ④ 'Spot 제어' 서버 (책꽂이 공정 · ARM STUDIO 다중 로봇, :7910) (2026-11-06)
#   이미 떠 있으면 아무것도 안 한다. Node 는 miniforge 의 spotnode 환경 (시스템 apt 는 안 건드림).
#   HOST=0.0.0.0 → 노트북에서도 http://192.168.137.29:7910 · MULTI_ROBOT=spot → 처음부터 Spot
APP="$HOME/2026_project/arm_studio_multi"
NODE="$HOME/miniforge3/envs/spotnode/bin/node"
ss -tln 2>/dev/null | grep -q ':7910 ' && exit 0
[ -x "$NODE" ] && [ -f "$APP/server.mjs" ] || { echo "Spot 제어: node 또는 server.mjs 없음" >&2; exit 0; }
cd "$APP" || exit 0
# 단일 명령으로 배경 실행 (cd && … & 꼴은 하위 셸이 부른 쪽의 출력을 붙잡아 ssh·상위 스크립트가 안 끝난다)
HOST=0.0.0.0 MULTI_ROBOT=spot PORT=7910 setsid nohup "$NODE" server.mjs > /tmp/spot_studio.log 2>&1 < /dev/null &
echo "Spot 제어 서버 시작 (:7910, 기록 /tmp/spot_studio.log)"
