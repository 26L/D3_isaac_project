"""
00_create_simple_cow_proxy_and_udder_markers.py

초기 단계에서 실제 cow USD/prim이 없을 때 사용한 proxy scene 생성용 스크립트입니다.
Isaac Sim Script Editor에서 실행합니다.

목적:
- 실제 소 asset이 없을 때, 단순한 cow proxy body와 유방/젖꼭지 marker를 만들어
  카메라 시야, 유방 검출 위치, 접근 목표 계산을 테스트합니다.
- 이후 실제 cow USD가 준비되면서 이 proxy는 실제 소 모델로 대체되었습니다.

생성 prim:
- /World/proxy_cow
- /World/proxy_cow/body
- /World/proxy_cow/udder
- /World/proxy_cow/teat_0~3
"""

from pxr import UsdGeom, Gf, UsdShade, Sdf
import omni.usd

stage = omni.usd.get_context().get_stage()

ROOT = "/World/proxy_cow"


def make_mat(path, color, roughness=0.45):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


body_mat = make_mat("/World/proxy_cow_body_mat", [0.78, 0.78, 0.72])
red_mat = make_mat("/World/proxy_cow_teat_red_mat", [1.0, 0.0, 0.0])

# Root
root = stage.DefinePrim(ROOT, "Xform")
xf = UsdGeom.Xformable(root)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.8))

# Body: sphere scaled to ellipsoid
body = UsdGeom.Sphere.Define(stage, ROOT + "/body")
body.CreateRadiusAttr(1.0)
body_xf = UsdGeom.Xformable(body.GetPrim())
body_xf.ClearXformOpOrder()
body_xf.AddScaleOp().Set(Gf.Vec3d(1.3, 0.45, 0.55))
UsdShade.MaterialBindingAPI(body.GetPrim()).Bind(body_mat)

# Udder: underside gray sphere
udder = UsdGeom.Sphere.Define(stage, ROOT + "/udder")
udder.CreateRadiusAttr(0.23)
udder_xf = UsdGeom.Xformable(udder.GetPrim())
udder_xf.ClearXformOpOrder()
udder_xf.AddTranslateOp().Set(Gf.Vec3d(-0.15, 0.0, -0.50))
udder_xf.AddScaleOp().Set(Gf.Vec3d(1.0, 0.85, 0.65))
UsdShade.MaterialBindingAPI(udder.GetPrim()).Bind(body_mat)

# Four red teat markers
teat_positions = [
    (-0.30, -0.16, -0.72),
    (-0.30,  0.16, -0.72),
    ( 0.05, -0.16, -0.72),
    ( 0.05,  0.16, -0.72),
]
for i, p in enumerate(teat_positions):
    teat = UsdGeom.Sphere.Define(stage, f"{ROOT}/teat_{i}")
    teat.CreateRadiusAttr(0.07)
    t_xf = UsdGeom.Xformable(teat.GetPrim())
    t_xf.ClearXformOpOrder()
    t_xf.AddTranslateOp().Set(Gf.Vec3d(*p))
    UsdShade.MaterialBindingAPI(teat.GetPrim()).Bind(red_mat)

print("[OK] simple cow proxy + udder/teat markers created")
print("     root:", ROOT)
