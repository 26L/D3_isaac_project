"""
02_print_marker_world_positions.py

수동 배치한 cow marker 4개의 world position을 확인하는 진단 코드입니다.
Isaac Sim Script Editor에서 실행합니다.
"""

from pxr import UsdGeom
import omni.usd
import numpy as np

stage = omni.usd.get_context().get_stage()

PATHS = [
    "/World/cow_head_marker",
    "/World/cow_tail_marker",
    "/World/cow_left_hind_marker",
    "/World/cow_right_hind_marker",
    "/World/b_plan_goal_marker",
    "/World/spot_with_arm",
    "/World/spot_with_arm/base",
]


def get_world_pos(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None

    xform = UsdGeom.Xformable(prim)
    mat = xform.ComputeLocalToWorldTransform(0.0)
    t = mat.ExtractTranslation()
    return np.array([t[0], t[1], t[2]], dtype=float)


print("==== world positions ====")
for p in PATHS:
    print(p, get_world_pos(p))
