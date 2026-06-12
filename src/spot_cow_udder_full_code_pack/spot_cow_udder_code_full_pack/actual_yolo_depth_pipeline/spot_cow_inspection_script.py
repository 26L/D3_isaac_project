"""
spot_cow_inspection_script.py  ── Isaac Sim Script Editor에서 실행
==================================================================
cow_yolo_3d_nav.py 가 계산한 nav_target.json 읽어서 Spot 이동.
UsdSkel 미사용 — 완전 YOLO + Depth 기반 네비게이션.

사전 조건:
  1. cow_capture_depth_script.py  실행 → /tmp/cow_capture.png, /tmp/cow_depth.npy
  2. cow_yolo_3d_nav.py           실행 → /tmp/nav_target.json
  3. Isaac Sim ▶ Play 상태

실행: Script Editor > File > Open > 이 파일 > Run
"""

import asyncio
import json
import math
import os

import omni.kit.app
import omni.kit.viewport.utility as vu
import omni.usd
from pxr import Gf, UsdGeom

# ── CONFIG ────────────────────────────────────────────────────────────────────
SPOT_PATH    = "/World/spot_with_arm"
REALSENSE    = "/World/spot_with_arm/arm0_link_wr1/Realsense/RSD455/Camera_OmniVision_OV9782_Color"
NAV_JSON     = "/tmp/nav_target.json"
CAPTURE_PATH = "/tmp/udder_capture.png"

ARM_STOW   = [ 1.57, -1.57,  1.57,  0.0,  0.0,  0.0]
ARM_READY  = [ 0.0,  -1.00,  1.80,  0.0,  0.0, -1.0]
ARM_INSERT = [ 0.0,  -0.30,  2.00,  0.0,  0.0, -1.57]
ARM_JOINTS = ["arm0_sh0","arm0_sh1","arm0_el0","arm0_el1","arm0_wr0","arm0_wr1"]

WAIT_MOVE  = 60
WAIT_ARM   = 40
WAIT_CAP   = 15

# ── HELPERS ───────────────────────────────────────────────────────────────────
async def wait(n):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


def teleport_spot(x, y, z, yaw_deg):
    stage = omni.usd.get_context().get_stage()
    prim  = stage.GetPrimAtPath(SPOT_PATH)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(x, y, z))
    half = math.radians(yaw_deg) / 2.0
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(math.cos(half), 0.0, 0.0, math.sin(half)))
    print(f"[SPOT] teleport -> ({x:.3f}, {y:.3f}, {z:.3f})  yaw={yaw_deg:.1f}deg")


def set_arm(positions):
    try:
        from omni.isaac.core.articulations import Articulation
        from omni.isaac.core.utils.types import ArticulationAction
        import numpy as np
        art     = Articulation(SPOT_PATH)
        art.initialize()
        dof     = list(art.dof_names)
        indices = [dof.index(n) for n in ARM_JOINTS if n in dof]
        valid   = [positions[i] for i, n in enumerate(ARM_JOINTS) if n in dof]
        art.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=np.array(valid),
                               joint_indices=np.array(indices)))
        print(f"[ARM] set -> {[round(p,2) for p in positions]}")
    except Exception as e:
        print(f"[ARM] error: {e}")


# ── MAIN ──────────────────────────────────────────────────────────────────────
async def run():
    print("\n=== INSPECTION PIPELINE START ===")

    # P1: nav_target.json 로드
    if not os.path.exists(NAV_JSON):
        print(f"[P1] ERROR: {NAV_JSON} 없음")
        print("     순서: cow_capture_depth_script.py → cow_yolo_3d_nav.py → 이 스크립트")
        return

    nav = json.load(open(NAV_JSON))
    src = nav.get("source", "unknown")
    dep = "depth" if nav.get("depth_used") else "ground-plane-fallback"
    print(f"[P1] nav loaded  source={src}  method={dep}")
    print(f"[P1] tail_3d = {[round(v,3) for v in nav['tail_3d']]}")
    print(f"[P1] head_3d = {[round(v,3) for v in nav['head_3d']]}")

    # P2: 목표 좌표 읽기
    tx     = nav["target_x"]
    ty     = nav["target_y"]
    tz     = nav["target_z"]
    yaw    = nav["yaw_deg"]
    print(f"[P2] target=({tx:.3f},{ty:.3f},{tz:.3f})  yaw={yaw:.1f}deg")

    # P3: Spot 이동
    print("[P3] moving Spot...")
    teleport_spot(tx, ty, tz, yaw)
    await wait(WAIT_MOVE)
    print("[P3] done")

    # P4: 팔 READY
    print("[P4] arm READY")
    set_arm(ARM_READY)
    await wait(WAIT_ARM)

    # P5: 팔 INSERT
    print("[P5] arm INSERT")
    set_arm(ARM_INSERT)
    await wait(WAIT_ARM * 2)
    print("[P5] done")

    # P6: RealSense 캡처
    print("[P6] RealSense capture...")
    vp = vu.get_active_viewport()
    prev = vp.camera_path
    vp.camera_path = REALSENSE
    await wait(WAIT_CAP)
    vu.capture_viewport_to_file(vp, CAPTURE_PATH)
    await wait(10)
    if os.path.exists(CAPTURE_PATH):
        print(f"[P6] saved: {CAPTURE_PATH} ({os.path.getsize(CAPTURE_PATH)//1024}KB)")
    vp.camera_path = prev

    # P7: 팔 회수
    print("[P7] retracting...")
    set_arm(ARM_READY)
    await wait(WAIT_ARM)
    set_arm(ARM_STOW)
    await wait(WAIT_ARM)
    print("[P7] done")

    print("\n=== PIPELINE COMPLETE ===")
    print(f"  nav source : {src} ({dep})")
    print(f"  capture    : {CAPTURE_PATH}")


asyncio.ensure_future(run())
