#!/usr/bin/env bash
# ============================================================
#  밤샘 계산 — 닭 머리 규칙을 '제대로' 고친 뒤 표를 새로 만든다
#
#  ★ 무엇이 바뀌었나 ★
#    예전 표는 노즐이 늘 정면(-Y)만 보게 만들어져서, 팔이 돌 때마다
#    j5(손목)로 되돌렸다. 실측 결과 예외 없이 j1+j5=0 이었다.
#    = "팔목이 꺾이면 안 된다" 는 규칙을 정면으로 어기고 있었고,
#      로봇 뒤쪽 공간도 통째로 막혀 있었다.
#    이제는 팔이 도는 만큼 주둥이도 같이 돈다(닭이 몸을 돌리듯).
#    손목은 안 쓰고(실측 j5 최대 0.01도), 뒤쪽까지 전부 열린다.
#
#  1) UR15 표 새로 만들기 (전 범위)      약 2.5시간
#  2) UR15 구멍 메우기                   약 30분
#  3) SO-ARM 표 만들기 (실물 로봇!)      약 1.5시간
#  4) SO-ARM 구멍 메우기                 약 20분
#
#  중간에 꺼져도 저장돼 있어 이어서 됩니다.
#  진행 상황:  tail -f /tmp/overnight.log
# ============================================================
cd "$(dirname "$0")"
source ~/anaconda3/etc/profile.d/conda.sh
conda activate gyu_arm

UR15=/home/drboom/2026_project/urdf/ur15_20260823/urdf/ur15_20260823.urdf
SOARM=/home/drboom/2026_project/urdf/sts3215/Industry_gyu_arm_20260809_base/urdf/Industry_gyu_arm_20260809_base.urdf
LOG=/tmp/overnight.log

say(){ { echo ""
         echo "=================================================="
         echo "  $*"
         echo "  $(date '+%m/%d %H:%M:%S')"
         echo "=================================================="; } | tee -a "$LOG"; }

if curl -s -o /dev/null --max-time 2 http://localhost:7900/ ; then
  echo "★ 앱이 켜져 있습니다. 먼저 끄고 다시 실행하세요."
  exit 1
fi

: > "$LOG"
say "시작 — 끝나면 '전부 완료' 가 뜹니다"
cp base_urdf.txt /tmp/base_urdf_overnight.bak

# ---------- 1) UR15 전 범위 ----------
say "1/4  UR15 표 새로 만들기 (전 범위 · 뒤쪽 포함)"
echo "$UR15" > base_urdf.txt
python build_head_gpu.py 1500 --hours=2.5 2>&1 | tee -a "$LOG"

# ---------- 2) UR15 구멍 ----------
say "2/4  UR15 표의 구멍 메우기"
python fill_head_holes.py 3 40 2>&1 | tee -a "$LOG"

# ---------- 3) SO-ARM (실물 로봇) ----------
say "3/4  SO-ARM 표 만들기 (실물 로봇 — 팔이 짧아 600mm 면 충분)"
echo "$SOARM" > base_urdf.txt
python build_head_gpu.py 600 --hours=1.5 2>&1 | tee -a "$LOG"

# ---------- 4) SO-ARM 구멍 ----------
say "4/4  SO-ARM 표의 구멍 메우기"
python fill_head_holes.py 3 40 2>&1 | tee -a "$LOG"

cp /tmp/base_urdf_overnight.bak base_urdf.txt
say "전부 완료 — 앱을 켜면 두 로봇 다 표가 준비돼 있습니다"
