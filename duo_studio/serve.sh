#!/bin/bash
# DUO STUDIO 띄우기 — 이 컴퓨터에서만 (127.0.0.1). 브라우저로 http://127.0.0.1:8772  (?robot=spot 이면 Spot 으로 시작, 책꽂이 공정은 /books.html)
#   켤 때: 이 포트를 잡고 있는 예전 웹 서버(python http.server)가 있으면 먼저 끈다.
#          다른 프로그램이 쓰고 있으면 끄지 않고 알려 주고 멈춘다.
#   끌 때(Ctrl+C): 서버를 끄고 포트가 비었는지 확인한다.
cd "$(dirname "$0")/web"
PORT=${PORT:-8772}

holders() { ss -ltnpH "sport = :$PORT" 2>/dev/null | grep -o 'pid=[0-9]*' | cut -d= -f2 | sort -u; }
stop_pid() {                                   # 먼저 TERM, 2초 안에 안 꺼지면 KILL
  kill "$1" 2>/dev/null
  for _ in $(seq 20); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.1; done
  kill -9 "$1" 2>/dev/null
}
for pid in $(holders); do
  cmd=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null)
  if [[ "$cmd" == *http.server* ]]; then
    echo "포트 $PORT 를 쓰던 예전 서버를 끕니다 (번호 $pid: $cmd)"
    stop_pid "$pid"
  else
    echo "포트 $PORT 를 다른 프로그램이 쓰고 있습니다 (번호 $pid: ${cmd:-알 수 없음}) — 끄지 않고 멈춥니다. PORT=다른번호 ./serve.sh 로 바꿀 수 있습니다." >&2
    exit 1
  fi
done
if [ -n "$(ss -ltnH "sport = :$PORT" 2>/dev/null)" ]; then
  echo "포트 $PORT 가 아직 안 비었습니다 (다른 사용자 프로그램일 수 있음) — 멈춥니다." >&2; exit 1
fi

python3 -m http.server "$PORT" --bind 127.0.0.1 &
SERVER=$!
cleanup() {
  trap - INT TERM EXIT
  stop_pid "$SERVER"; wait "$SERVER" 2>/dev/null
  if [ -n "$(ss -ltnH "sport = :$PORT" 2>/dev/null)" ]; then echo "포트 $PORT 가 아직 열려 있습니다" >&2; else echo "꺼짐 — 포트 $PORT 정리됨"; fi
}
trap cleanup INT TERM EXIT
echo "DUO STUDIO → http://127.0.0.1:$PORT   (끄기: Ctrl+C)"
wait "$SERVER"
