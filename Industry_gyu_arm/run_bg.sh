#!/bin/bash
# 서버를 '완전히 분리해서' 띄운다.
#   터미널이나 접속 세션이 끊겨도 서버는 계속 산다.
#   (그냥 nohup 만으로는 부모 프로세스가 죽을 때 같이 죽는 경우가 있다)
cd "$(dirname "$0")"
pkill -f 'python.*arm_studio_pro\.py' 2>/dev/null
sleep 2
setsid nohup ./start_pro.sh > /tmp/start_out.log 2>&1 < /dev/null &
echo "서버 시작 중... (로그: tail -f /tmp/arm_studio.log)"
for i in $(seq 1 30); do
    sleep 2
    if curl -s --max-time 2 -o /dev/null http://localhost:7900/ 2>/dev/null; then
        echo "준비 완료 → http://localhost:7900"
        exit 0
    fi
done
echo "시작 실패 — /tmp/start_out.log 를 확인하세요"
exit 1
