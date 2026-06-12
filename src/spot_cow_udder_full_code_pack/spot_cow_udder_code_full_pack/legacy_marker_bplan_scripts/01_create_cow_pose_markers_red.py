"""
01_create_cow_pose_markers_red.py

실제 cow USD가 들어온 뒤, B안 geometry planner를 테스트하기 위해
소의 기준점 marker 4개를 만드는 스크립트입니다.

Isaac Sim Script Editor에서 실행합니다.

Marker:
- /World/cow_head_marker
- /World/cow_tail_marker
- /World/cow_left_hind_marker
- /World/cow_right_hind_marker
"""

from pxr import UsdGeom, Gf, UsdShade, Sdf
import omni.usd

stage = omni.usd.get_context().get_stage()

MARKERS = {
    "/World/cow_head_marker":       {"pos": [-1.0, 0.0, 1.2]},
    "/World/cow_tail_marker":       {"pos": [ 0.8, 0.0, 1.0]},
    "/World/cow_left_hind_marker":  {"pos": [ 0.8, 0.25, 0.25]},
    "/World/cow_right_hind_marker": {"pos": [ 0.8,-0.25, 0.25]},
}

RED = [1.0, 0.0, 0.0]


def create_material(mat_path, color):
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))
    )
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.2)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def create_marker(path, xyz, radius=0.07):
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(radius)

    prim = sphere.GetPrim()
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()

    t = xform.AddTranslateOp()
    t.Set(Gf.Vec3d(float(xyz[0]), float(xyz[1]), float(xyz[2])))

    mat = create_material(path + "_red_mat", RED)
    UsdShade.MaterialBindingAPI(prim).Bind(mat)

    print(f"[MARKER] {path} at {xyz}")


for path, cfg in MARKERS.items():
    create_marker(path, cfg["pos"])

print("[OK] all cow pose markers created in red")
