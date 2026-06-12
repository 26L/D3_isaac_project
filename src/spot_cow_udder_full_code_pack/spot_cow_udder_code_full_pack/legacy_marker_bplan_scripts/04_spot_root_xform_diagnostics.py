"""
04_spot_root_xform_diagnostics.py

Spot root transform op가 꼬였을 때 확인하는 진단 스크립트입니다.
Isaac Sim Script Editor에서 실행합니다.
"""

from pxr import UsdGeom
import omni.usd

stage = omni.usd.get_context().get_stage()
ROOT_PATH = "/World/spot_with_arm"

prim = stage.GetPrimAtPath(ROOT_PATH)
if not prim.IsValid():
    raise RuntimeError(f"Invalid path: {ROOT_PATH}")

xform = UsdGeom.Xformable(prim)

print("==== SPOT ROOT XFORM OPS ====")
for op in xform.GetOrderedXformOps():
    print(op.GetOpName(), op.Get())
