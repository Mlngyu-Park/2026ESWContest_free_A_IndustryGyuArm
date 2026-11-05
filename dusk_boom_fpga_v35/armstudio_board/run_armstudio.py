#!/usr/bin/env python3
"""run_armstudio.py - start ARM STUDIO PRO on the DUSK_BOOM board (same as `python arm_studio_pro.py`)
after the board-only patches (see /opt/dusk/py: gradio stand-in, fast_simplification in numpy,
collision checks without python-fcl).  The ARM STUDIO files themselves are not changed."""
import os
import runpy
import sys

APP = os.environ.get("ARM_STUDIO_DIR", "/home/drboom/2026_project/Industry_gyu_arm")
sys.path.insert(0, "/opt/dusk/py")
sys.path.insert(0, APP)
os.chdir(APP)

import dusk_board_patches  # noqa: E402

dusk_board_patches.apply()

# 두 팔: 원본은 다시 켤 때 calib.json 의 다른 팔 자리(arm_pose)를 읽지 않는다 (불러오는 목록에 없다)
#   -> 보드에서는 켤 때 한 번 되살려 A·B 두 팔이 바로 보이게 한다 (원본 파일은 그대로)
import json  # noqa: E402
import arm_studio_web as core  # noqa: E402

try:
    _d = json.load(open(core.PROJECT_FILE))
    if _d.get("arm_pose") and not core.S.get("arm_pose"):
        core.S["arm_pose"] = _d["arm_pose"]
        print("[dusk] 두 팔 자리 되살림:", _d["arm_pose"], flush=True)
except Exception as _e:
    print("[dusk] arm_pose 되살리기 실패:", _e, flush=True)

sys.argv = [os.path.join(APP, "arm_studio_pro.py")]
runpy.run_path(sys.argv[0], run_name="__main__")
