"""
cow_capture_depth_script.py  -- Isaac Sim Script Editor
Left + Right + Color 캡처 → 터미널에서 OpenCV 스테레오 depth 계산
step_async 완전 미사용
"""
import asyncio, json, os
import numpy as np
import omni.kit.app
import omni.kit.viewport.utility as vu
import omni.usd
from pxr import UsdGeom

LEFT_PATH  = "/tmp/cow_left.png"
RIGHT_PATH = "/tmp/cow_right.png"
RGB_PATH   = "/tmp/cow_capture.png"
CAMINFO    = "/tmp/camera_info.json"
IMG_W, IMG_H = 1280, 720

BASE = "/World/spot_with_arm/arm0_link_wr1/Realsense/RSD455"
CAM_COLOR = f"{BASE}/Camera_OmniVision_OV9782_Color"
CAM_LEFT  = f"{BASE}/Camera_OmniVision_OV9782_Left"
CAM_RIGHT = f"{BASE}/Camera_OmniVision_OV9782_Right"


async def capture():
    app = omni.kit.app.get_app()
    vp  = vu.get_active_viewport()

    async def shoot(cam_path, out_path, label):
        # 이전 파일 삭제 (0KB 잔재 방지)
        if os.path.exists(out_path):
            os.remove(out_path)
        vp.camera_path = cam_path
        for _ in range(40):          # 카메라 전환 + 렌더 안정화
            await app.next_update_async()
        vu.capture_viewport_to_file(vp, out_path)
        for _ in range(30):          # 파일 쓰기 완료 대기
            await app.next_update_async()
        sz = os.path.getsize(out_path) if os.path.exists(out_path) else 0
        print(f"[CAP] {label} -> {out_path}  ({sz//1024}KB)")
        return sz > 0

    # 카메라 경로 존재 확인
    stage = omni.usd.get_context().get_stage()
    for path, label in [(CAM_COLOR,"Color"), (CAM_LEFT,"Left"), (CAM_RIGHT,"Right")]:
        ok = stage.GetPrimAtPath(path).IsValid()
        print(f"[CAP] prim 확인: {label} → {'OK' if ok else 'NOT FOUND'}")
        if not ok and label == "Color":
            print(f"[CAP] FATAL: Color 카메라 경로 없음 → 경로 확인 필요\n  {path}")
            return

    print("[CAP] 시작...")
    ok_c = await shoot(CAM_COLOR, RGB_PATH,   "Color")
    ok_l = await shoot(CAM_LEFT,  LEFT_PATH,  "Left ")
    ok_r = await shoot(CAM_RIGHT, RIGHT_PATH, "Right")

    if not ok_c:
        print("[CAP] ERROR: Color 캡처 실패 — Play 버튼 확인")
        return
    if not (ok_l and ok_r):
        print("[CAP] WARN: Left/Right 캡처 실패 → stereo 없이 진행 (ground-plane fallback 사용)")

    # 카메라 파라미터
    cam_prim = stage.GetPrimAtPath(CAM_COLOR)

    fl, h_ap, v_ap = 50.0, 36.0, 20.25
    if cam_prim.IsValid():
        c    = UsdGeom.Camera(cam_prim)
        fl   = c.GetFocalLengthAttr().Get()        or fl
        h_ap = c.GetHorizontalApertureAttr().Get() or h_ap
        v_ap = c.GetVerticalApertureAttr().Get()   or v_ap
    fx = fl * IMG_W / h_ap
    fy = fl * IMG_H / v_ap

    # 카메라 월드 포즈
    wm = None
    try:
        from omni.isaac.core.prims import XFormPrim
        xfp = XFormPrim(prim_path=CAM_COLOR)
        pos, quat = xfp.get_world_pose()
        tx, ty, tz = float(pos[0]), float(pos[1]), float(pos[2])
        w, x, y, z = float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])
        R  = np.array([
            [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
            [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
            [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
        Rt = R.T
        wm  = [
            [float(Rt[0,0]), float(Rt[0,1]), float(Rt[0,2]), 0.0],
            [float(Rt[1,0]), float(Rt[1,1]), float(Rt[1,2]), 0.0],
            [float(Rt[2,0]), float(Rt[2,1]), float(Rt[2,2]), 0.0],
            [tx, ty, tz, 1.0],
        ]
        print(f"[CAP] cam pos: ({tx:.3f}, {ty:.3f}, {tz:.3f})")
    except Exception as e:
        print(f"[CAP] XFormPrim 실패: {e}")
        mat = UsdGeom.XformCache().GetLocalToWorldTransform(cam_prim)
        wm  = [list(mat.GetRow(i)) for i in range(4)]

    # 스테레오 기준선 (Left↔Right 실제 거리)
    baseline = 0.095   # RSD455 기본값 95mm; 아래서 USD로 계산 시도
    try:
        from omni.isaac.core.prims import XFormPrim
        lp = XFormPrim(prim_path=CAM_LEFT).get_world_pose()[0]
        rp = XFormPrim(prim_path=CAM_RIGHT).get_world_pose()[0]
        baseline = float(np.linalg.norm(np.array(lp) - np.array(rp)))
        print(f"[CAP] 스테레오 baseline: {baseline*100:.1f}cm")
    except Exception:
        print(f"[CAP] baseline 기본값: {baseline*100:.1f}cm")

    cam_info = {
        "width": IMG_W, "height": IMG_H,
        "fx": fx, "fy": fy,
        "cx": IMG_W / 2.0, "cy": IMG_H / 2.0,
        "camera_path": CAM_COLOR,
        "world_matrix": wm,
        "stereo_baseline": baseline,
        "left_path":  LEFT_PATH,
        "right_path": RIGHT_PATH,
        "cow_world_pos": None,
    }
    with open(CAMINFO, "w") as f:
        json.dump(cam_info, f, indent=2)
    print(f"[CAP] CamInfo -> {CAMINFO}  fx={fx:.1f}  baseline={baseline*100:.1f}cm")
    print("[CAP] 완료! -> bash run_full_pipeline_3d.sh")


asyncio.ensure_future(capture())
