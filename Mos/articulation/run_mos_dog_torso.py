"""加载 Mos/assets/mos_dog.usd（torso + 4 legs 的整狗），按 trot 步态驱动。

整只狗是一个 articulation，torso 为漂浮根。每条腿的 hip_motor 通过 FixedJoint
固定在 torso 角点上。

运行前先用 compose_mos_dog.py 生成 mos_dog.usd：
    python3 Mos/articulation/compose_mos_dog.py
然后：
    ./isaaclab.sh -p Mos/articulation/run_mos_dog_torso.py
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Drive the assembled mos_dog.")
parser.add_argument(
    "--usd",
    type=str,
    default=os.path.join(os.path.dirname(__file__), "..", "assets", "mos_dog.usd"),
    help="Path to the composed dog USD.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import math
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationContext


# 单腿的关节命名延续 mos_0509 单腿脚本。
DRIVEN_PER_LEG = ["hip_joint", "font_gear", "rear_gear"]
PASSIVE_PER_LEG = ["shank_middle", "shank_link", "thigh_gear", "shank_gear"]

# Trot 步态：对角同相 (FR↔BL)，另一对 180° 偏置 (FL↔BR)。
LEG_PHASES = {
    "FR": 0.0,
    "FL": math.pi,
    "BR": math.pi,
    "BL": 0.0,
}

# 单腿步态参数 (amp, freq, intra_leg_joint_phase)，沿用 run_mos_0509.py。
JOINT_PARAMS = {
    "hip_joint": (0.25, 0.20, 0.0),
    "rear_gear": (-0.40, 0.15, math.pi / 4),
    "font_gear": (0.40, -0.15, math.pi / 2),
}

INIT_TORSO_POS = (0.0, 0.0, 0.65)  # 比 USD 里 torso 默认 z=0.6 略高一点，留落地余量


def make_cfg(usd_path: str) -> ArticulationCfg:
    # joint_names_expr 用 regex 匹配 4 条腿同名的关节：
    # 实际 short name 在 articulation.data.joint_names 里通常会带腿前缀（FR_ / FL_ / ...）
    # 如果不带前缀就重复，IsaacLab 会按 prim path 区分但 find_joints 会用 regex 匹配所有。
    driven_re = [f".*{n}$" for n in DRIVEN_PER_LEG]
    passive_re = [f".*{n}$" for n in PASSIVE_PER_LEG]

    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.abspath(usd_path),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=100.0,
                enable_gyroscopic_forces=True,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=64,
                solver_velocity_iteration_count=4,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=INIT_TORSO_POS,
        ),
        actuators={
            "driven": ImplicitActuatorCfg(
                joint_names_expr=driven_re,
                effort_limit_sim=30.0,
                velocity_limit_sim=3.0,
                stiffness=20.0,
                damping=2.0,
            ),
            "passive": ImplicitActuatorCfg(
                joint_names_expr=passive_re,
                effort_limit_sim=0.0,
                velocity_limit_sim=1000.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )


def design_scene(usd_path: str):
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    sim_utils.create_prim("/World/DogOrigin", "Xform", translation=(0.0, 0.0, 0.0))

    dog_cfg = make_cfg(usd_path)
    dog_cfg.prim_path = "/World/DogOrigin/Dog"
    return Articulation(cfg=dog_cfg)


def classify_joints(joint_names):
    """Return mapping: leg_name -> {short_joint_name -> joint_index}."""
    legs = {"FR": {}, "FL": {}, "BR": {}, "BL": {}}
    for idx, full in enumerate(joint_names):
        for leg in legs:
            if leg in full:
                # 取 short name：取最后一段 short joint 名（hip_joint / font_gear / ...）
                for jn in (*DRIVEN_PER_LEG, *PASSIVE_PER_LEG):
                    if full.endswith(jn) or jn in full:
                        legs[leg][jn] = idx
                        break
                break
    return legs


def run(sim: SimulationContext, dog: Articulation):
    sim_dt = sim.get_physics_dt()
    t = 0.0
    count = 0

    print(f"[INFO] Joint names ({len(dog.data.joint_names)}):")
    for i, n in enumerate(dog.data.joint_names):
        print(f"  [{i:2d}] {n}")
    print(f"[INFO] Body names ({len(dog.data.body_names)}):")
    for i, n in enumerate(dog.data.body_names):
        print(f"  [{i:2d}] {n}")

    leg_joint_idx = classify_joints(dog.data.joint_names)
    print("[INFO] Per-leg driven indices:")
    for leg in ("FR", "FL", "BR", "BL"):
        print(f"  {leg}: " + ", ".join(
            f"{j}->{leg_joint_idx[leg].get(j)}" for j in DRIVEN_PER_LEG
        ))

    while simulation_app.is_running():
        if count % 1000 == 0:
            joint_pos = dog.data.default_joint_pos.clone()
            joint_vel = dog.data.default_joint_vel.clone()
            dog.write_joint_state_to_sim(joint_pos, joint_vel)
            dog.reset()
            t = 0.0
            print("[INFO] Reset.")

        # 构建全关节目标向量（按 articulation 内部索引）。被动关节保持默认 0。
        n_joints = dog.data.joint_names.__len__()
        targets_full = torch.zeros((dog.num_instances, n_joints), device=dog.device)

        for leg in ("FR", "FL", "BR", "BL"):
            body_phase = LEG_PHASES[leg]
            for jname in DRIVEN_PER_LEG:
                idx = leg_joint_idx[leg].get(jname)
                if idx is None:
                    continue
                amp, freq, joint_phase = JOINT_PARAMS[jname]
                targets_full[:, idx] = amp * math.sin(2.0 * math.pi * freq * t + joint_phase + body_phase)

        dog.set_joint_position_target(targets_full)
        dog.write_data_to_sim()

        sim.step()
        dog.update(sim_dt)
        t += sim_dt
        count += 1


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([1.5, 1.5, 1.0], [0.0, 0.0, 0.4])

    dog = design_scene(args_cli.usd)

    sim.reset()
    print("[INFO] Setup complete.")
    run(sim, dog)


if __name__ == "__main__":
    main()
    simulation_app.close()
