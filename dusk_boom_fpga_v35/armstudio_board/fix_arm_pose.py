# fix_arm_pose.py - put the B arm position (laptop calib.json, 11-03) back into the board copy
import json, shutil
P = "/home/drboom/2026_project/urdf/sts3215/Industry_gyu_arm_20260728_base/calib.json"
shutil.copy(P, P + ".before_armpose")
d = json.load(open(P))
d["arm_pose"] = {"B": [0.30000000000000016, -0.48, 1.5707963267948966]}
json.dump(d, open(P, "w"), indent=1)
print("ARM_POSE_FIXED", d["arm_pose"])
