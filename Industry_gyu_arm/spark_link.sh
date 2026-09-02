#!/bin/bash
# DGX Spark 연결: LLM(:8000->18000) + Whisper(:8090->18090)
#   경로: devtunnel(비공개 터널) -> Spark SSH -> 포트 전달
#   start_pro.sh 가 자동 호출. 실패해도 앱은 노트북 LLM/규칙 파서로 잘 돈다.
# 터널 주소는 저장소에 넣지 않는다 - spark_tunnel.txt (한 줄) 에 넣을 것
TUNNEL=$(cat "$(dirname "$0")/spark_tunnel.txt" 2>/dev/null | head -1)
[ -n "$TUNNEL" ] || { echo "[Spark] spark_tunnel.txt 없음 - 건너뜀"; exit 0; }
DT=$HOME/.local/bin/devtunnel
LOG=/tmp/devtunnel_connect.log

# ① devtunnel 클라이언트 (Spark 포트들을 이 노트북으로)
if ! pgrep -f "devtunnel connect $TUNNEL" >/dev/null 2>&1; then
    [ -x "$DT" ] || { echo "[Spark] devtunnel 없음 - 건너뜀"; exit 0; }
    rm -f "$LOG"
    nohup "$DT" connect "$TUNNEL" > "$LOG" 2>&1 &
fi

# ② SSH(22)가 몇 번 로컬 포트로 열렸는지 로그에서 찾는다 (매번 달라짐!)
P=""
for i in $(seq 1 24); do
    P=$(grep -o 'from 127.0.0.1:[0-9]* to host port 22\.' "$LOG" 2>/dev/null \
        | tail -1 | grep -o ':[0-9]*' | tr -d ':')
    # ★ tail -1 ★ devtunnel 이 재연결되면 로그에 포트가 계속 쌓인다.
    #   head -1 은 '맨 처음(이미 죽은)' 포트를 잡아 늘 연결 실패했다.
    #   실제로 열려 있는지도 확인해서, 아니면 로그를 거꾸로 훑는다.
    if [ -n "$P" ] && ! ss -tln 2>/dev/null | grep -q "127.0.0.1:$P "; then
        for _p in $(grep -o 'from 127.0.0.1:[0-9]* to host port 22\.' "$LOG" \
                    | grep -o ':[0-9]*' | tr -d ':' | tac); do
            if ss -tln 2>/dev/null | grep -q "127.0.0.1:$_p "; then P=$_p; break; fi
        done
    fi
    [ -n "$P" ] && break
    sleep 0.5
done
[ -n "$P" ] || { echo "[Spark] 터널에서 SSH 포트를 못 찾음 - 노트북 모드"; exit 0; }

# ③ LLM + Whisper 포트 터널
if ! pgrep -f "ssh.*-L 18090:127.0.0.1:8090" >/dev/null 2>&1; then
    pkill -f "ssh.*-L 18000:127.0.0.1:8000" 2>/dev/null   # 옛 터널 정리
    sleep 0.5
    ssh -f -N -i "$HOME/.ssh/spark_key" -p "$P" \
        -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
        -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 \
        -L 18000:127.0.0.1:8000 -L 18090:127.0.0.1:8090 drboom@127.0.0.1 2>/dev/null \
      && echo "[Spark] LLM(18000)+Whisper(18090) 연결됨 (SSH 포트 $P)" \
      || { echo "[Spark] SSH 연결 실패 - 노트북 모드"; exit 0; }
else
    echo "[Spark] LLM+Whisper 터널 이미 연결됨"
fi

# ④ Spark 의 whisper-server(:8090)가 죽어 있으면 살린다 (음성 인식용)
if ! curl -s --max-time 3 http://127.0.0.1:18090/ -o /dev/null 2>/dev/null; then
    echo "[Spark] Whisper 꺼져 있음 -> 기동"
    ssh -i "$HOME/.ssh/spark_key" -p "$P" -o ConnectTimeout=8 drboom@127.0.0.1 \
        'cd ~/project/2026_project/whisper.cpp && \
         setsid nohup ./build/bin/whisper-server \
            -m models/ggml-large-v3-turbo.bin --host 127.0.0.1 --port 8090 -l ko \
            > /tmp/whisper-server.log 2>&1 < /dev/null &' 2>/dev/null
    for i in $(seq 1 20); do
        curl -s --max-time 2 http://127.0.0.1:18090/ -o /dev/null 2>/dev/null \
            && { echo "[Spark] Whisper 준비됨"; break; }
        sleep 2
    done
fi
