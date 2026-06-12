"""
05_remove_yaw_delta_op.py

yaw 자동 정렬 실험 중 추가한 xformOp:rotateZ:yaw_delta가 있으면 제거하는 복구 스크립트입니다.
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
ops = list(xform.GetOrderedXformOps())

print("==== BEFORE OPS ====")
for op in ops:
    print(op.GetOpName(), op.Get())

new_ops = []
removed = []
for op in ops:
    if op.GetOpName() == "xformOp:rotateZ:yaw_delta":
        removed.append(op)
    else:
        new_ops.append(op)

xform.SetXformOpOrder(new_ops)

print("==== REMOVED OPS ====")
for op in removed:
    print(op.GetOpName(), op.Get())

print("==== AFTER OPS ====")
for op in xform.GetOrderedXformOps():
    print(op.GetOpName(), op.Get())

print("[OK] yaw_delta op removed if it existed")
