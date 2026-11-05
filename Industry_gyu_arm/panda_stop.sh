#!/bin/bash
# 라떼판다 바탕화면 'ARM STUDIO 끄기' 아이콘 (2026-11-04) — 뒤에서 도는 ARM STUDIO 서버를 끈다 (브라우저 창은 따로 닫기)
ICON="$(dirname "$(readlink -f "$0")")/arm_studio_icon.png"
P=$(pgrep -f 'python.*arm_studio_pro\.py')
if [ -z "$P" ]; then
    command -v notify-send >/dev/null && notify-send -i "$ICON" "ARM STUDIO" "이미 꺼져 있습니다"; exit 0
fi
command -v zenity >/dev/null && ! zenity --question --title="ARM STUDIO" --text="ARM STUDIO 를 끌까요?\n(실행 중인 작업이 있으면 멈춥니다)" 2>/dev/null && exit 0
kill $P
for i in $(seq 1 20); do pgrep -f 'python.*arm_studio_pro\.py' >/dev/null || break; sleep 0.5; done
command -v notify-send >/dev/null && notify-send -i "$ICON" "ARM STUDIO" "껐습니다"
