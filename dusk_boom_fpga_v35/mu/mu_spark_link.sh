#!/bin/bash
# mu_spark_link.sh — 뮤(라떼판다)가 노트북을 거쳐 DGX Spark 의 Whisper(음성 인식)를 쓰게 한다 (2026-11-05)
#
#   뮤 127.0.0.1:18090 ──(윈도우 ssh.exe -R)──> 노트북 127.0.0.1:18090 ──(WSL ssh -L, devtunnel)──> 스파크 8090
#
#   - 넘기는 것은 Whisper 하나뿐 (말로 만들기는 뮤의 더블 레이첼, LLM 포트는 넘기지 않는다)
#   - 뮤에는 스파크 열쇠를 두지 않는다 (노트북이 이미 가진 접속만 쓴다)
#   - 끊기면 둘 다 5초 뒤 다시 잇는다. devtunnel 이 다시 붙어 SSH 포트가 바뀌어도 찾아간다.
#   실행: bash mu_spark_link.sh start | stop | status
HERE="$(dirname "$(readlink -f "$0")")"
TUNNEL="$(head -1 "$HOME/2026_project/Industry_gyu_arm/spark_tunnel.txt" 2>/dev/null)"
DT="$HOME/.local/bin/devtunnel"
DTLOG=/tmp/mu_devtunnel.log
MU="${MU_IP:-192.168.137.29}"
KEY_WIN='C:\Users\BrDoo\.ssh\latte_key'
PIDF=/tmp/mu_spark_link.pids
SSHO=(-o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=15 -o ServerAliveCountMax=3
      -o ExitOnForwardFailure=yes)

spark_port() {   # devtunnel 이 스파크 SSH(22)를 연 로컬 포트 — 실제로 SSH 가 되는 것
    local pid p
    pid=$(pgrep -f "devtunnel connect $TUNNEL" | head -1)
    [ -n "$pid" ] || return 1
    for p in $(ss -tlnp 2>/dev/null | grep "pid=$pid," | grep -o '127.0.0.1:[0-9]*' | cut -d: -f2 | sort -u); do
        timeout 10 ssh "${SSHO[@]}" -o "HostKeyAlias=[127.0.0.1]:34529" -i "$HOME/.ssh/spark_key" -p "$p" \
            drboom@127.0.0.1 true 2>/dev/null && { echo "$p"; return 0; }
    done
    return 1
}

loop_spark() {   # 노트북 127.0.0.1:18090 -> 스파크 127.0.0.1:8090
    while true; do
        if ! pgrep -f "devtunnel connect $TUNNEL" >/dev/null; then
            nohup "$DT" connect "$TUNNEL" > "$DTLOG" 2>&1 &
            sleep 8
        fi
        P=$(spark_port)
        if [ -n "$P" ]; then
            echo "$(date '+%F %T') 스파크 SSH 포트 $P 로 Whisper 연결"
            ssh -N "${SSHO[@]}" -o "HostKeyAlias=[127.0.0.1]:34529" -i "$HOME/.ssh/spark_key" -p "$P" \
                -L 127.0.0.1:18090:127.0.0.1:8090 drboom@127.0.0.1
            echo "$(date '+%F %T') 스파크 연결 끊김 — 다시"
        else
            echo "$(date '+%F %T') devtunnel 에서 스파크 SSH 를 못 찾음 — devtunnel 다시 띄움"
            pkill -f "devtunnel connect $TUNNEL"; sleep 2
        fi
        sleep 5
    done
}

loop_mu() {      # 뮤 127.0.0.1:18090 -> (윈도우) 127.0.0.1:18090 = WSL 의 위 터널
    while true; do
        echo "$(date '+%F %T') 뮤($MU) 로 18090 넘김"
        ssh.exe -N -o BatchMode=yes -o ConnectTimeout=8 -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
            -o ExitOnForwardFailure=yes -i "$KEY_WIN" -R 127.0.0.1:18090:127.0.0.1:18090 "drboom@$MU" 2>&1
        echo "$(date '+%F %T') 뮤 연결 끊김 — 다시"
        sleep 5
    done
}

case "${1:-start}" in
start)
    [ -f "$PIDF" ] && "$0" stop >/dev/null
    nohup setsid bash -c "$(declare -f spark_port loop_spark); $(declare -p TUNNEL DT DTLOG SSHO); loop_spark" \
        >> /tmp/mu_spark_link_spark.log 2>&1 < /dev/null &
    echo $! > "$PIDF"
    sleep 2
    nohup setsid bash -c "$(declare -f loop_mu); $(declare -p MU KEY_WIN); loop_mu" \
        >> /tmp/mu_spark_link_mu.log 2>&1 < /dev/null &
    echo $! >> "$PIDF"
    echo "시작: 기록 /tmp/mu_spark_link_spark.log · /tmp/mu_spark_link_mu.log"
    ;;
stop)
    if [ -f "$PIDF" ]; then
        for p in $(cat "$PIDF"); do pkill -TERM -s "$p" 2>/dev/null; kill "$p" 2>/dev/null; done
        rm -f "$PIDF"
    fi
    pkill -f -- "-L 127.0.0.1:18090:127.0.0.1:8090" 2>/dev/null
    pkill -f -- "-R 127.0.0.1:18090:127.0.0.1:18090" 2>/dev/null
    echo "멈춤"
    ;;
status)
    curl -s -m 3 -o /dev/null -w "노트북 18090 (스파크 Whisper): HTTP %{http_code}\n" http://127.0.0.1:18090/
    tail -2 /tmp/mu_spark_link_spark.log /tmp/mu_spark_link_mu.log 2>/dev/null
    ;;
esac
