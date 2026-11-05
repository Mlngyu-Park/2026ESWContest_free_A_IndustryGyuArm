#!/bin/bash
# ARM STUDIO — M68K 공장 3D (모토로라 68000 렌더러)
#
#   ./start_m68k.sh
#
# start_pro.sh 와 별개의 실행 명령이다. start_pro.sh 는 건드리지 않는다.
# 다른 점은 두 가지뿐:
#   1) 떠 있던 것을 무조건 싹 끄고 완전히 새로 띄운다
#   2) 켜지면 68000 화면(scene.bin)까지 만들어 놓는다
#
# 종료: Ctrl+C
cd "$(dirname "$0")"

APP_PORT=7900
VIEW_PORT=7870

# ── 1. 떠 있던 것 전부 종료 ─────────────────────────────────
#   앱 본체 + conda 껍데기 + 68000 렌더러 + RACHEL 까지.
#   렌더러는 앱이 띄운 자식이라 앱만 죽이면 좀비로 남는다.
#   start_pro.sh 로 띄운 것도 여기서 같이 끈다 — 포트(7900/7870)를
#   하나만 쓸 수 있으니 둘이 같이 뜰 수 없다. 어느 쪽으로 켰든
#   이 명령은 '완전히 새로 시작' 이다.
echo "▸ 이전에 떠 있던 것 정리"
pkill -f 'start_pro\.sh'               2>/dev/null
pkill -f 'python.*arm_studio_pro\.py'  2>/dev/null
pkill -f 'python.*arm_studio_web\.py'  2>/dev/null
pkill -f 'conda run.*arm_studio'       2>/dev/null
pkill -f 'm68k_array/gfx/pl68k'        2>/dev/null
pkill -f 'rachel_cpp/rachel'           2>/dev/null

# 포트가 풀릴 때까지 기다린다
for i in $(seq 1 20); do
    ss -tln 2>/dev/null | grep -qE ":($APP_PORT|$VIEW_PORT)\s" || break
    sleep 0.5
done
# 그래도 물고 있으면 그 프로세스를 직접 끊는다 (포트 충돌로 안 뜨는 것 방지)
if ss -tln 2>/dev/null | grep -qE ":($APP_PORT|$VIEW_PORT)\s"; then
    echo "  포트가 안 풀려 강제 종료"
    for port in $APP_PORT $VIEW_PORT; do
        for pid in $(ss -tlnp 2>/dev/null | grep ":$port " \
                     | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u); do
            kill -9 "$pid" 2>/dev/null
        done
    done
    sleep 1
fi

# ── 2. 68000 렌더러 빌드 (소스가 더 새로우면) ───────────────
GFX=m68k_array/gfx
if [ -f "$GFX/pl68k.c" ]; then
    if [ ! -x "$GFX/pl68k" ] || [ "$GFX/pl68k.c" -nt "$GFX/pl68k" ]; then
        echo "▸ 68000 렌더러 빌드"
        ( cd "$GFX" && gcc -O2 -march=native -o pl68k pl68k.c -lm ) \
            && echo "  완료" || echo "  실패 — 68000 화면은 안 나옵니다"
    fi
fi

bash spark_link.sh || true      # DGX Spark 두뇌 연결 (안 되면 자동 대체)

# ── 3. 앱 실행 ──────────────────────────────────────────────
#   start_pro.sh 는 부르지 않는다. 파이썬을 직접 띄운다.
#   (start_pro.sh 를 부르면 그 안의 정리 로직이 또 돌아서 꼬인다)
#   로그도 따로 쓴다 — start_pro.sh 것과 섞이면 원인 추적이 안 된다.
LOG=/tmp/arm_studio_m68k.log
: > "$LOG"
ln -sf "$LOG" ./arm_studio_m68k.log 2>/dev/null || true
echo "▸ 로그: $LOG   (tail -f $LOG)"

# 이 셸에서 start_pro.sh 를 실행하지 못하게 막는다.
# 실수로든 다른 스크립트가 부르든, 여기서는 절대 안 돈다.
start_pro.sh() { echo "start_m68k.sh 안에서는 start_pro.sh 를 쓰지 않습니다"; return 1; }
export ARM_STUDIO_LAUNCHER=m68k

stdbuf -oL -eL conda run --no-capture-output -n gyu_arm \
    python -u arm_studio_pro.py 2>&1 | tee -a "$LOG" &
APP_PID=$!

# ── 4. 켜지면 68000 장면을 구워 둔다 ────────────────────────
(
    for i in $(seq 1 60); do
        code=$(curl -s --max-time 2 -o /dev/null -w '%{http_code}' \
               "http://127.0.0.1:$APP_PORT/" 2>/dev/null)
        [ "$code" = "200" ] && break
        sleep 2
    done
    [ "$code" = "200" ] || exit 0

    # 앱은 켤 때마다 공장을 비운다(플레이트만). 그대로 구우면 바닥만 나오니
    # 쓰던 배치를 먼저 올린다. 없으면 '이전 배치'로 넘어간다.
    for preset in "한이음_대상" "한이음 대상" "이전 배치"; do
        ok=$(curl -s --max-time 30 -X POST "http://127.0.0.1:$APP_PORT/api/call" \
             -H 'Content-Type: application/json' \
             -d "$(printf '{"fn":"factory_load","args":["%s"]}' "$preset")" \
             2>/dev/null | grep -c '불러옴')
        if [ "$ok" = "1" ]; then
            echo "▸ 공장 배치: $preset"
            break
        fi
    done

    echo ""
    echo "▸ 68000 장면 굽는 중 (10초쯤 걸립니다)"
    curl -s --max-time 200 -X POST \
         "http://127.0.0.1:$APP_PORT/api/m68k/rebuild?tri=350" -o /dev/null 2>/dev/null
    echo "▸ 68000 공장 3D:  http://localhost:$APP_PORT/m68k/viewer.html"
    echo ""
) &

echo ""
echo "  ARM STUDIO       http://localhost:$APP_PORT"
echo "  68000 공장 3D    http://localhost:$APP_PORT/m68k/viewer.html"
echo "  (Ctrl+C 로 종료)"
echo ""

wait $APP_PID
