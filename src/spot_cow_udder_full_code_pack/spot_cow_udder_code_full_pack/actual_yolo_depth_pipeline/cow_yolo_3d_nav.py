#!/usr/bin/env python3
"""
cow_yolo_3d_nav.py  ── 터미널에서 실행
======================================
YOLO 2D 키포인트 + Stereo/Depth → 3D 역투영 → Spot 네비게이션 목표 계산.
UsdSkel 미사용. 완전 YOLO + 카메라 기반.

실행:
  /home/rokey/dev_ws/venv/isaaclab/bin/python3 cow_yolo_3d_nav.py

입력:
  /tmp/cow_capture.png     RGB  (cow_capture_depth_script.py 출력)
  /tmp/cow_left.png        Left stereo  (선택)
  /tmp/cow_right.png       Right stereo (선택)
  /tmp/cow_depth.npy       Depth float32 meter (선택, 없으면 stereo→fallback)
  /tmp/camera_info.json    카메라 파라미터

출력:
  /tmp/nav_target.json     Spot 목표 좌표 + 방향
  /tmp/cow_3d_result.png   시각화
"""

import json
import math
import os
import sys

import cv2
import numpy as np

WEIGHTS    = "/home/rokey/Downloads/yolo/best.pt"
RGB_PATH   = "/tmp/cow_capture.png"
DEPTH_PATH = "/tmp/cow_depth.npy"
CAMINFO    = "/tmp/camera_info.json"
NAV_OUT    = "/tmp/nav_target.json"
VIS_OUT    = "/tmp/cow_3d_result.png"
STEREO_DEPTH_PATH = "/tmp/cow_stereo_depth.npy"

CONF_DET   = 0.35
CONF_KP    = 0.50
REAR_DIST  = 1.5   # 소 뒤에서 Spot이 서는 거리 (m)
SPOT_Z     = 0.6   # Spot 기립 높이 (m)

KP_NAMES = [
    "nose","head","withers","back","sacrum","tail_base",
    "fl_knee","fr_knee","hl_hock","hr_hock",
    "fl_hoof","fr_hoof","hl_hoof","hr_hoof",
]


# ══════════════════════════════════════════════════════════════════════════════
# ■ 역투영 (2D pixel + depth → 3D world)
# ══════════════════════════════════════════════════════════════════════════════

def backproject(u, v, depth_m, cam_info):
    """
    픽셀 (u, v) + 거리 depth_m → 월드 3D 좌표.

    USD 카메라 관례:
      로컬 프레임: X=오른쪽, Y=위, -Z=전방(씬 안쪽)
      image v=0 → 위, v=H → 아래 (Y 반전 필요)
    """
    fx = cam_info["fx"]
    fy = cam_info["fy"]
    cx = cam_info["cx"]
    cy = cam_info["cy"]

    # 카메라 로컬 좌표 (Z-forward convention)
    Xc =  (u - cx) / fx * depth_m
    Yc = -(v - cy) / fy * depth_m   # 이미지 Y축 반전 (USD Y-up)
    Zc = -depth_m                   # USD 카메라는 -Z 방향 전방

    # 월드 변환 (4×4) — USD row-vector 방식: local @ M
    M  = np.array(cam_info["world_matrix"])  # camera_local → world
    pw = np.array([Xc, Yc, Zc, 1.0]) @ M
    return pw[:3]


def get_depth_at(depth_img, u, v, radius=3):
    """
    픽셀 주변 radius 범위 median depth (outlier 제거).
    유효값(0 < D < 1e5)만 사용.
    """
    h, w = depth_img.shape[:2]
    u0, u1 = max(0, u - radius), min(w, u + radius + 1)
    v0, v1 = max(0, v - radius), min(h, v + radius + 1)
    patch = depth_img[v0:v1, u0:u1].flatten()
    valid = patch[(patch > 0.05) & (patch < 1e5)]
    if len(valid) == 0:
        return None
    return float(np.median(valid))


# ══════════════════════════════════════════════════════════════════════════════
# ■ 스테레오 Depth 계산
# ══════════════════════════════════════════════════════════════════════════════

def compute_stereo_depth(left_path, right_path, cam_info):
    """
    Left + Right 스테레오 이미지 → depth map (float32, meter)
    Isaac Sim RSD455: 수평 baseline, 완전 정렬 → 이미 rectified.
    """
    left  = cv2.imread(left_path,  cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
    if left is None or right is None:
        print(f"[STEREO] 이미지 로드 실패: {left_path}, {right_path}")
        return None

    fx       = cam_info["fx"]
    baseline = cam_info.get("stereo_baseline", 0.095)

    # Isaac Sim에서 Left가 오른쪽에 보이는 경우 swap
    # (Left camera가 scene 기준 오른쪽에 있으면 disparity 부호 반전)
    block_sz = 11
    num_disp = 128

    sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block_sz,
        P1=8  * 3 * block_sz ** 2,
        P2=32 * 3 * block_sz ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.StereoSGBM_MODE_SGBM_3WAY
    )

    disp16 = sgbm.compute(left, right).astype(np.float32) / 16.0

    # disparity가 음수 또는 0이면 Left/Right 순서가 반대 → swap 재시도
    pos_ratio = (disp16 > 1.0).mean()
    if pos_ratio < 0.1:
        print("[STEREO] disparity 음수 지배적 → Left/Right swap 재시도")
        disp16 = sgbm.compute(right, left).astype(np.float32) / 16.0

    depth = np.zeros_like(disp16, dtype=np.float32)
    valid = disp16 > 1.0
    depth[valid] = fx * baseline / disp16[valid]
    depth[depth > 30.0] = 0.0   # 30m 이상 무효
    depth[depth < 0.1]  = 0.0   # 10cm 미만 무효

    valid_px = int((depth > 0).sum())
    if valid_px < 100:
        print(f"[STEREO] 유효 픽셀 너무 적음({valid_px}px) → fallback 사용")
        return None

    med = float(np.median(depth[depth > 0]))
    print(f"[STEREO] depth 완료  valid={valid_px}px  median={med:.2f}m  "
          f"baseline={baseline*100:.1f}cm  fx={fx:.0f}")
    return depth


# ══════════════════════════════════════════════════════════════════════════════
# ■ YOLO 추론
# ══════════════════════════════════════════════════════════════════════════════

def run_yolo(img_bgr):
    from ultralytics import YOLO
    model = YOLO(WEIGHTS)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    results = model(img_rgb, conf=CONF_DET, verbose=False)
    dets = []
    for r in results:
        if r.keypoints is None:
            continue
        boxes = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        kpts  = r.keypoints.data.cpu().numpy()
        for i in range(len(boxes)):
            dets.append({"bbox": boxes[i].tolist(),
                         "conf": float(confs[i]),
                         "kpts": kpts[i].tolist()})
    # 가장 큰 소 선택
    if not dets:
        return None
    dets.sort(key=lambda d: (d["bbox"][2]-d["bbox"][0])*(d["bbox"][3]-d["bbox"][1]),
              reverse=True)
    return dets[0]


# ══════════════════════════════════════════════════════════════════════════════
# ■ 메인
# ══════════════════════════════════════════════════════════════════════════════

def main():
    # ── 파일 확인 ────────────────────────────────────────────────────────────
    for path in [RGB_PATH, CAMINFO]:
        if not os.path.exists(path):
            print(f"[3DNAV] ERROR: 없음 → {path}")
            sys.exit(1)

    cam_info = json.load(open(CAMINFO))
    depth_img = None
    depth_source = "none"

    # 1순위: 이미 계산된 depth .npy
    if os.path.exists(DEPTH_PATH):
        depth_img    = np.load(DEPTH_PATH)
        depth_source = "npy_file"
        print(f"[3DNAV] depth 로드: {DEPTH_PATH}")

    # 2순위: 스테레오 depth 계산 (캐시 있으면 재사용)
    elif os.path.exists(STEREO_DEPTH_PATH):
        depth_img    = np.load(STEREO_DEPTH_PATH)
        depth_source = "stereo_cached"
        print(f"[3DNAV] stereo depth 캐시 로드: {STEREO_DEPTH_PATH}")

    # 3순위: Left/Right 이미지로 새로 계산
    else:
        lp = cam_info.get("left_path",  "/tmp/cow_left.png")
        rp = cam_info.get("right_path", "/tmp/cow_right.png")
        if os.path.exists(lp) and os.path.exists(rp):
            print("[3DNAV] 스테레오 depth 계산 중...")
            depth_img = compute_stereo_depth(lp, rp, cam_info)
            if depth_img is not None:
                np.save(STEREO_DEPTH_PATH, depth_img)
                depth_source = "stereo_computed"
                print(f"[3DNAV] stereo depth 저장: {STEREO_DEPTH_PATH}")
        else:
            print(f"[3DNAV] 스테레오 이미지 없음({lp}, {rp}) → ground-plane fallback")

    if depth_img is None:
        print("[3DNAV] WARNING: depth 없음 → ground-plane fallback 사용")

    depth_available = depth_img is not None

    # ── 로드 ─────────────────────────────────────────────────────────────────
    img = cv2.imread(RGB_PATH)

    print(f"[3DNAV] image  {img.shape[1]}x{img.shape[0]}")
    print(f"[3DNAV] camera fx={cam_info['fx']:.1f}  fy={cam_info['fy']:.1f}")
    if depth_img is not None:
        valid_mask = (depth_img > 0.05) & (depth_img < 1e5)
        print(f"[3DNAV] depth  source={depth_source}  shape={depth_img.shape}  "
              f"valid={valid_mask.sum()}px  "
              f"median={np.median(depth_img[valid_mask]):.2f}m")

    # ── YOLO 탐지 ────────────────────────────────────────────────────────────
    print("[3DNAV] YOLO 추론 중...")
    det = run_yolo(img)
    if det is None:
        print("[3DNAV] ERROR: 소 미탐지")
        sys.exit(1)
    print(f"[3DNAV] 소 탐지 conf={det['conf']:.2f}")

    # 키포인트 픽셀 좌표 수집
    kps_2d = {}
    for i, name in enumerate(KP_NAMES):
        kp = det["kpts"][i]
        u, v, c = int(kp[0]), int(kp[1]), float(kp[2])
        if c >= CONF_KP:
            kps_2d[name] = (u, v, c)

    print(f"[3DNAV] 유효 키포인트: {list(kps_2d.keys())}")

    # ── 3D 계산 ─────────────────────────────────────────────────────────────
    M   = np.array(cam_info["world_matrix"])
    fx  = cam_info["fx"];  fy = cam_info["fy"]
    cx  = cam_info["cx"];  cy = cam_info["cy"]
    kps_3d = {}

    cow_wp = cam_info.get("cow_world_pos")  # Isaac Sim에서 저장한 소 위치

    # ── 방법 A: Hybrid (소 위치=stage, 방향=YOLO) ────────────────────────────
    if cow_wp is not None:
        cow_center = np.array(cow_wp, dtype=float)
        print(f"[3DNAV] 소 위치 (stage): ({cow_center[0]:.3f}, {cow_center[1]:.3f}, {cow_center[2]:.3f})")

        head_kp = kps_2d.get("head") or kps_2d.get("nose")
        tail_kp = kps_2d.get("tail_base") or kps_2d.get("sacrum")
        if head_kp is None or tail_kp is None:
            print("[3DNAV] ERROR: head 또는 tail 키포인트 없음")
            sys.exit(1)

        h_u, h_v, _ = head_kp
        t_u, t_v, _ = tail_kp

        # 이미지 방향 → 월드 방향 (깊이 독립)
        # 이미지 +U = 카메라 local +X → 월드 M[0,:3]
        # 이미지 +V = 카메라 local -Y → 월드 -M[1,:3]
        du = float(t_u - h_u)
        dv = float(t_v - h_v)
        d_world = du / fx * M[0, :3] - dv / fy * M[1, :3]
        d_world[2] = 0.0
        norm = np.linalg.norm(d_world)
        if norm < 1e-4:
            print("[3DNAV] ERROR: head-tail 방향 추출 실패 (픽셀 같음)")
            sys.exit(1)
        tail_dir = d_world / norm

        # tail/head 위치 추정 (소 중심 기준 ±0.7m)
        tail_3d = cow_center + tail_dir * 0.7
        head_3d = cow_center - tail_dir * 0.7
        print(f"[3DNAV] tail_dir (YOLO): ({tail_dir[0]:.3f}, {tail_dir[1]:.3f})")
        print(f"[3DNAV] tail_3d (est) : ({tail_3d[0]:.3f}, {tail_3d[1]:.3f}, {tail_3d[2]:.3f})")
        src_label = "hybrid(stage+YOLO)"

    # ── 방법 B: Depth 역투영 ─────────────────────────────────────────────────
    elif depth_img is not None:
        for name, (u, v, c) in kps_2d.items():
            D = get_depth_at(depth_img, u, v)
            if D is not None:
                world_pt = backproject(u, v, D, cam_info)
                kps_3d[name] = world_pt
                print(f"  {name:12s}  px=({u},{v})  D={D:.2f}m  "
                      f"world=({world_pt[0]:.2f},{world_pt[1]:.2f},{world_pt[2]:.2f})")
        tail_3d = kps_3d.get("tail_base")
        if tail_3d is None:
            tail_3d = kps_3d.get("sacrum")
        head_3d = kps_3d.get("head")
        if head_3d is None:
            head_3d = kps_3d.get("nose")
        if head_3d is None:
            head_3d = kps_3d.get("withers")
        if tail_3d is None or head_3d is None:
            print("[3DNAV] ERROR: tail/head depth 없음")
            sys.exit(1)
        tail_dir = tail_3d - head_3d
        tail_dir[2] = 0.0
        tail_dir /= max(np.linalg.norm(tail_dir), 1e-4)
        src_label = "depth_backprojection"

    # ── 방법 C: Ground-plane fallback ────────────────────────────────────────
    else:
        print("[3DNAV] Ground-plane fallback: Z=0 교차 계산")
        cam_origin = M[3, 0:3].copy()
        print(f"[3DNAV] cam_origin = ({cam_origin[0]:.3f}, {cam_origin[1]:.3f}, {cam_origin[2]:.3f})")
        if abs(cam_origin[2]) < 0.05:
            print("[3DNAV] WARN: 카메라 Z≈0 → 기본 높이 1.0m 적용")
            cam_origin[2] = 1.0
        for name, (u, v, c) in kps_2d.items():
            ray_local = np.array([(u-cx)/fx, -(v-cy)/fy, -1.0])
            ray_local /= np.linalg.norm(ray_local)
            ray_world = ray_local @ M[:3, :3]
            ray_world /= np.linalg.norm(ray_world)
            if abs(ray_world[2]) < 1e-6:
                continue
            t = -cam_origin[2] / ray_world[2]
            if t > 0:
                kps_3d[name] = cam_origin + t * ray_world
        tail_3d = kps_3d.get("tail_base")
        if tail_3d is None:
            tail_3d = kps_3d.get("sacrum")
        head_3d = kps_3d.get("head")
        if head_3d is None:
            head_3d = kps_3d.get("nose")
        if head_3d is None:
            head_3d = kps_3d.get("withers")
        if tail_3d is None or head_3d is None:
            print("[3DNAV] ERROR: ground-plane 교차 실패")
            sys.exit(1)
        tail_dir = tail_3d - head_3d
        tail_dir[2] = 0.0
        tail_dir /= max(np.linalg.norm(tail_dir), 1e-4)
        src_label = "ground_plane_fallback"

    print(f"\n[3DNAV] tail_3d = ({tail_3d[0]:.3f}, {tail_3d[1]:.3f}, {tail_3d[2]:.3f})")
    print(f"[3DNAV] head_3d = ({head_3d[0]:.3f}, {head_3d[1]:.3f}, {head_3d[2]:.3f})")

    # ── Spot 목표 계산 ───────────────────────────────────────────────────────
    target = tail_3d + tail_dir * REAR_DIST
    target[2] = SPOT_Z

    # Spot이 소를 바라보는 yaw
    face_dir = -tail_dir
    yaw_deg  = math.degrees(math.atan2(face_dir[1], face_dir[0]))

    print(f"\n[3DNAV] ── 네비게이션 목표 ──")
    print(f"  tail_dir  : ({tail_dir[0]:.3f}, {tail_dir[1]:.3f})")
    print(f"  target XYZ: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")
    print(f"  yaw       : {yaw_deg:.1f} deg")

    # ── 저장 ─────────────────────────────────────────────────────────────────
    nav = {
        "target_x":   float(target[0]),
        "target_y":   float(target[1]),
        "target_z":   float(target[2]),
        "yaw_deg":    yaw_deg,
        "tail_3d":    tail_3d.tolist(),
        "head_3d":    head_3d.tolist(),
        "tail_dir":   tail_dir.tolist(),
        "source":     src_label,
        "depth_used": depth_available,
        "kps_3d":     {k: v.tolist() for k, v in kps_3d.items()},
    }
    with open(NAV_OUT, "w") as f:
        json.dump(nav, f, indent=2)
    print(f"\n[3DNAV] 저장 → {NAV_OUT}")

    # ── 시각화 ───────────────────────────────────────────────────────────────
    vis = img.copy()
    for name, (u, v, c) in kps_2d.items():
        color = (0, 255, 0) if name in kps_3d else (100, 100, 100)
        cv2.circle(vis, (u, v), 6, color, -1)
        cv2.putText(vis, name[:6], (u+5, v-4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)
    # tail / head 강조
    if "tail_base" in kps_2d:
        u, v, _ = kps_2d["tail_base"]
        cv2.circle(vis, (u,v), 14, (0, 60, 255), 3)
        cv2.putText(vis, "TAIL", (u+8, v-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,60,255), 2)
    if "head" in kps_2d:
        u, v, _ = kps_2d["head"]
        cv2.circle(vis, (u,v), 14, (255, 200, 0), 3)
        cv2.putText(vis, "HEAD", (u+8, v-12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,200,0), 2)

    depth_label = depth_source if depth_available else "FALLBACK(Z=0)"
    cv2.putText(vis, f"3D Nav | {depth_label}", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,128), 2)
    cv2.putText(vis, f"target:({target[0]:.2f},{target[1]:.2f})  yaw={yaw_deg:.1f}",
                (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,128), 2)

    cv2.imwrite(VIS_OUT, vis)
    print(f"[3DNAV] 시각화 → {VIS_OUT}")
    print(f"\n  다음: Isaac Sim Script Editor에서 spot_cow_inspection_script.py 실행")


if __name__ == "__main__":
    main()
