"""
03_b_plan_marker_rear_goal_demo.py

Marker 기반 B안 후방 접근 goal 계산 및 Spot base 이동 데모입니다.
Isaac Sim Script Editor에서 실행합니다.

핵심:
rear_dir = normalize(tail_marker - head_marker)
hind_center = (left_hind_marker + right_hind_marker) / 2
goal = hind_center + rear_dir * REAR_DISTANCE

주의:
- yaw 자동 정렬은 Spot root transform op를 깨뜨릴 수 있어 이 파일에서는 제외합니다.
- translate-only 방식으로 /World/spot_with_arm/base를 goal에 맞춥니다.
"""

import time
import numpy as np
from pxr import UsdGeom, Gf, UsdShade, Sdf
import omni.usd

stage = omni.usd.get_context().get_stage()

SPOT_ROOT_PATH = "/World/spot_with_arm"
SPOT_BASE_PATH = "/World/spot_with_arm/base"

HEAD_PATH = "/World/cow_head_marker"
TAIL_PATH = "/World/cow_tail_marker"
LH_PATH = "/World/cow_left_hind_marker"
RH_PATH = "/World/cow_right_hind_marker"

GOAL_PATH = "/World/b_plan_goal_marker"

REAR_DISTANCE = 0.70
GOAL_MARKER_Z = 0.15
GOAL_MARKER_RADIUS = 0.12

MOVE_STEPS = 120
MOVE_DT = 0.02


def normalize(v, eps=1e-8):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < eps:
        raise RuntimeError("zero vector")
    return v / n


def get_world_pos(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid path: {path}")

    xform = UsdGeom.Xformable(prim)
    mat = xform.ComputeLocalToWorldTransform(0.0)
    t = mat.ExtractTranslation()
    return np.array([t[0], t[1], t[2]], dtype=float)


def set_translate_keep_rotation(path, xyz):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Invalid path: {path}")

    xform = UsdGeom.Xformable(prim)
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            translate_op = op
            break

    if translate_op is None:
        translate_op = xform.AddTranslateOp()

    translate_op.Set(Gf.Vec3d(float(xyz[0]), float(xyz[1]), float(xyz[2])))


def create_red_material(path):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.0, 0.0))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.0, 0.0))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def set_goal_sphere(path, xyz, radius=GOAL_MARKER_RADIUS):
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(radius)

    prim = sphere.GetPrim()
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    t = xform.AddTranslateOp()
    t.Set(Gf.Vec3d(float(xyz[0]), float(xyz[1]), float(xyz[2])))

    mat = create_red_material(path + "_mat")
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def compute_and_create_rear_goal():
    head = get_world_pos(HEAD_PATH)
    tail = get_world_pos(TAIL_PATH)
    lh = get_world_pos(LH_PATH)
    rh = get_world_pos(RH_PATH)

    rear_dir = normalize(tail[:2] - head[:2])
    hind_center = (lh[:2] + rh[:2]) / 2.0
    hind_gap = np.linalg.norm(lh[:2] - rh[:2])

    if hind_gap < 0.15:
        raise RuntimeError("Hind leg markers are too close. Check marker placement.")

    goal_xy = hind_center + rear_dir * REAR_DISTANCE
    goal_xyz = np.array([goal_xy[0], goal_xy[1], GOAL_MARKER_Z])

    set_goal_sphere(GOAL_PATH, goal_xyz)

    print("========== B-PLAN GOAL ==========")
    print("head        :", head)
    print("tail        :", tail)
    print("left hind   :", lh)
    print("right hind  :", rh)
    print("rear_dir    :", rear_dir)
    print("hind_center :", hind_center)
    print("hind_gap    :", hind_gap)
    print("goal_xyz    :", goal_xyz)
    print("=================================")

    return goal_xyz, rear_dir


def move_base_to_goal():
    root_now = get_world_pos(SPOT_ROOT_PATH)
    base_now = get_world_pos(SPOT_BASE_PATH)
    goal = get_world_pos(GOAL_PATH)

    goal_base = np.array([goal[0], goal[1], base_now[2]])
    offset = base_now - root_now
    target_root = goal_base - offset

    print("========== MOVE BASE ==========")
    print("root_now   :", root_now)
    print("base_now   :", base_now)
    print("goal_base  :", goal_base)
    print("offset     :", offset)
    print("target_root:", target_root)
    print("===============================")

    for i in range(MOVE_STEPS + 1):
        a = i / MOVE_STEPS
        root_xyz = (1.0 - a) * root_now + a * target_root
        set_translate_keep_rotation(SPOT_ROOT_PATH, root_xyz)
        time.sleep(MOVE_DT)

    print("[OK] /World/spot_with_arm/base moved to rear goal marker")


compute_and_create_rear_goal()
move_base_to_goal()
