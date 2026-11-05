#!/bin/bash
# 라떼판다 바탕화면 'ARM STUDIO' 아이콘이 부르는 실행기 (2026-11-04)
#   ① 이미 켜져 있으면 브라우저만 연다
#   ② 아니면 서버를 뒤에서 켜고, 뜰 때까지 기다렸다가 브라우저로 http://localhost:7900 을 연다
#   서버 기록: /tmp/arm_studio.log (start_pro.sh 가 남긴다) · 끄기: 바탕화면 'ARM STUDIO 끄기'
cd "$(dirname "$(readlink -f "$0")")" || exit 1
export PATH="$HOME/miniforge3/bin:$PATH"
URL=http://localhost:7900
ICON="$PWD/arm_studio_icon.png"
up() { ss -tln 2>/dev/null | grep -q ':7900 '; }
note() { command -v notify-send >/dev/null && notify-send -i "$ICON" "ARM STUDIO" "$1"; }
if ! up; then
    note "켜는 중입니다… (20초쯤 걸립니다)"
    # ★ 모터 포트 권한(dialout) ★ 권한을 준 뒤 다시 로그인하지 않은 화면에서도 모터를 쓰도록 그룹을 붙여서 켠다
    if id -nG | grep -qw dialout || ! getent group dialout | grep -qw "$USER"; then
        setsid nohup ./start_pro.sh > /tmp/arm_studio_launch.log 2>&1 < /dev/null &
    else
        setsid nohup sg dialout -c ./start_pro.sh > /tmp/arm_studio_launch.log 2>&1 < /dev/null &
    fi
    for i in $(seq 1 90); do up && break; sleep 1; done
    if ! up; then
        command -v zenity >/dev/null && zenity --error --title="ARM STUDIO" \
            --text="켜지 못했습니다.\n기록: /tmp/arm_studio.log" 2>/dev/null
        exit 1
    fi
fi
xdg-open "$URL" >/dev/null 2>&1 &
