"""把 4 个 mos_0509.usd 的腿挂到一个 torso 上，组装成 Mos/assets/mos_dog.usd。

策略：
  - 新建 /dog/torso 作为漂浮 articulation 根（Box rigid body）
  - 把 mos_0509.usd reference 4 次，分别放到 /dog/legs/{FR,FL,BR,BL}
  - 改写每条腿原来的 root_joint：
      * body0 从 [] (world) 改为 /dog/torso
      * localPos0 设为该腿在 torso 框架下的角点偏移
      * 移除 ArticulationRootAPI / PhysxArticulationAPI（articulation 根改为 /dog/torso）

用法（无需起 Isaac Sim）：
    python3 Mos/articulation/compose_mos_dog.py
"""

from __future__ import annotations

import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


REPO = os.path.abspath(os.path.dirname(__file__) + "/../..")
LEG_USD = os.path.join(REPO, "Mos/assets/mos_0509.usd")
DOG_USD = os.path.join(REPO, "Mos/assets/mos_dog.usd")

# Body geometry (m). Body forward = +X, body left = +Y, up = +Z.
BODY_LENGTH = 0.40    # 前后髋间距
BODY_WIDTH = 0.20     # 左右髋间距
BODY_HEIGHT = 0.06    # torso 厚度（视觉占位）
TORSO_MASS = 5.0      # 躯干质量 (kg)
TORSO_Z = 0.6         # 初始离地高度 (m)

LEGS = [
    # (name, dx_in_torso_frame, dy_in_torso_frame)
    ("FR", +BODY_LENGTH / 2, -BODY_WIDTH / 2),
    ("FL", +BODY_LENGTH / 2, +BODY_WIDTH / 2),
    ("BR", -BODY_LENGTH / 2, -BODY_WIDTH / 2),
    ("BL", -BODY_LENGTH / 2, +BODY_WIDTH / 2),
]


def make_torso(stage: Usd.Stage, path: str) -> None:
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, TORSO_Z))
    xf.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    xf.AddScaleOp().Set(Gf.Vec3f(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT))

    prim = cube.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(TORSO_MASS)


def add_leg(stage: Usd.Stage, name: str, dx: float, dy: float, leg_usd_rel: str) -> None:
    leg_path = f"/dog/legs/{name}"
    leg_prim = stage.DefinePrim(leg_path, "Xform")
    leg_prim.GetReferences().AddReference(leg_usd_rel)

    # 把腿放到 torso 的对应角点上方（torso 中心在世界 z=TORSO_Z）。
    # 引用进来的 /mos_leg_51 自带 xformOp:translate=(0,0,0.4679...)。这里直接覆盖
    # xformOp:translate，让腿的整体位置精确落在 (dx, dy, TORSO_Z)，配合下面 root_joint
    # 的 localPos0 让 hip_motor 与 torso 角点对齐，避免 PhysX 起步纠位。
    leg_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(dx, dy, TORSO_Z))

    # 改写 root_joint：原本是 world->hip_motor 的 FixedJoint，且带 ArticulationRootAPI；
    # 现在让它变成 torso->hip_motor 的 FixedJoint，并把 articulation 根标志移除。
    rj_path = f"{leg_path}/root_joint"
    rj_prim = stage.GetPrimAtPath(rj_path)
    if not rj_prim.IsValid():
        raise RuntimeError(f"missing prim after reference: {rj_path}")

    joint = UsdPhysics.Joint(rj_prim)
    joint.CreateBody0Rel().SetTargets([Sdf.Path("/dog/torso")])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(dx, dy, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    # 移除原来的 articulation 根标志（articulation 根改为 /dog/torso）。
    rj_prim.RemoveAppliedSchema("PhysicsArticulationRootAPI")
    rj_prim.RemoveAppliedSchema("PhysxArticulationAPI")

    print(f"  leg {name}: torso->hip_motor at torso-local ({dx:+.3f}, {dy:+.3f}, 0)")


def main() -> None:
    if not os.path.exists(LEG_USD):
        raise SystemExit(f"missing leg USD: {LEG_USD}")

    stage = Usd.Stage.CreateNew(DOG_USD)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    dog = stage.DefinePrim("/dog", "Xform")
    stage.SetDefaultPrim(dog)
    UsdPhysics.ArticulationRootAPI.Apply(dog)

    make_torso(stage, "/dog/torso")
    print(f"[INFO] torso at z={TORSO_Z}, size=({BODY_LENGTH}, {BODY_WIDTH}, {BODY_HEIGHT}), mass={TORSO_MASS}")

    stage.DefinePrim("/dog/legs", "Xform")

    leg_usd_rel = os.path.relpath(LEG_USD, os.path.dirname(DOG_USD))
    print(f"[INFO] referencing leg USD: {leg_usd_rel}")
    for name, dx, dy in LEGS:
        add_leg(stage, name, dx, dy, leg_usd_rel)

    stage.GetRootLayer().Save()
    print(f"[INFO] saved {DOG_USD}")


if __name__ == "__main__":
    main()
