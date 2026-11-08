#!/bin/bash
# ARM STUDIO 다중 로봇 실행 (ARM STUDIO 프로 화면 복사판 — G1 · Spot · UR15)
#   ./start.sh   → 브라우저에서 http://localhost:7910
#   켤 때: 이 포트를 잡고 있던 예전 서버(이 폴더의 server.mjs)가 있으면 먼저 끈다. 다른 프로그램이면 알려 주고 멈춘다.
#   끌 때(Ctrl+C): 서버를 끄고 포트가 비었는지 확인한다.
cd "$(dirname "$0")"
PORT=${PORT:-7910}
holders() { ss -ltnpH "sport = :$PORT" 2>/dev/null | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u; }
stop_pid() { kill "$1" 2>/dev/null; for _ in $(seq 20); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.1; done; kill -9 "$1" 2>/dev/null; }
for pid in $(holders); do
  cmd=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null)
  if [[ "$cmd" == *server.mjs* ]]; then echo "포트 $PORT 를 쓰던 예전 서버를 끕니다 (번호 $pid)"; stop_pid "$pid"
  else echo "포트 $PORT 를 다른 프로그램이 쓰고 있습니다 (번호 $pid: ${cmd:-알 수 없음}) — 끄지 않고 멈춥니다. PORT=다른번호 ./start.sh" >&2; exit 1; fi
done
PORT=$PORT node server.mjs &
SERVER=$!
cleanup() { trap - INT TERM EXIT; stop_pid "$SERVER"; wait "$SERVER" 2>/dev/null
  if [ -n "$(ss -ltnH "sport = :$PORT" 2>/dev/null)" ]; then echo "포트 $PORT 가 아직 열려 있습니다" >&2; else echo "꺼짐 — 포트 $PORT 정리됨"; fi; }
trap cleanup INT TERM EXIT
wait "$SERVER"
