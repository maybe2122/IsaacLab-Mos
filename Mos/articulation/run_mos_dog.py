"""在 Isaac Sim 中加载 4 条 mos_0509.usd，按矩形 4 角拼成一只"狗"。

每条腿是一个独立的 Articulation，hip_motor 通过 USD 自带的 root_joint 固定在
世界的对应位置。这只"狗"没有躯干、走不动，但能直观看到 4 条腿的相位/形态。

Usage:
    ./isaaclab.sh -p Mos/articulation/run_mos_dog.py
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn 4 mos_0509 legs as a static quadruped.")
parser.add_argument(
    "--usd",
    type=str,
    default=os.path.join(os.path.dirname(__file__), "..", "assets", "mos_0509.usd"),
    help="Path to the mos_0509 USD.",
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


DRIVEN_JOINTS = ["hip_joint", "font_gear", "rear_gear"]
PASSIVE_JOINTS = ["shank_middle", "shank_link", "thigh_gear", "shank_gear"]

# 整机几何（m）：body 朝向 +X，左侧 +Y，向上 +Z。
BODY_LENGTH = 0.40   # 前后 hip 距离
BODY_WIDTH = 0.20    # 左右 hip 距离
HIP_HEIGHT = 0.6     # 髋关节离地高度

# Trot 步态：对角 (FR↔BL) 同相，另一对 (FL↔BR) 偏 180°。
# 单腿步态相位 (hip / rear_gear / font_gear) 沿用原 run_mos_0509.py 设计。
LEGS = [
    # name,  x,                  y,                  body_phase
    ("FR", +BODY_LENGTH / 2, -BODY_WIDTH / 2, 0.0),
    ("FL", +BODY_LENGTH / 2, +BODY_WIDTH / 2, math.pi),
    ("BR", -BODY_LENGTH / 2, -BODY_WIDTH / 2, math.pi),
    ("BL", -BODY_LENGTH / 2, +BODY_WIDTH / 2, 0.0),
]

# 单腿驱动参数 (amp, freq, intra_leg_joint_phase)，与 run_mos_0509.py 完全一致。
JOINT_PARAMS = {
    "hip_joint": (0.25, 0.20, 0.0),
    "rear_gear": (-0.40, 0.15, math.pi / 4),
    "font_gear": (0.40, -0.15, math.pi / 2),
}


def make_cfg(usd_path: str, init_pos) -> ArticulationCfg:
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
            pos=init_pos,
            joint_pos={n: 0.0 for n in DRIVEN_JOINTS},
        ),
        actuators={
            "driven": ImplicitActuatorCfg(
                joint_names_expr=DRIVEN_JOINTS,
                effort_limit_sim=30.0,
                velocity_limit_sim=3.0,
                stiffness=20.0,
                damping=2.0,
            ),
            "passive": ImplicitActuatorCfg(
                joint_names_expr=PASSIVE_JOINTS,
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

    sim_utils.create_prim("/World/Dog", "Xform", translation=(0.0, 0.0, 0.0))

    legs: dict[str, Articulation] = {}
    leg_phases: dict[str, float] = {}
    for name, x, y, phase in LEGS:
        leg_cfg = make_cfg(usd_path, init_pos=(x, y, HIP_HEIGHT))
        leg_cfg.prim_path = f"/World/Dog/{name}"
        legs[name] = Articulation(cfg=leg_cfg)
        leg_phases[name] = phase
        print(f"[INFO] Spawn {name} at ({x:+.3f}, {y:+.3f}, {HIP_HEIGHT}) phase={phase:.3f}")
    return legs, leg_phases


def run(sim: SimulationContext, legs: dict, leg_phases: dict):
    sim_dt = sim.get_physics_dt()
    t = 0.0
    count = 0

    leg_state = {}
    for name, leg in legs.items():
        joint_ids, joint_names = leg.find_joints(DRIVEN_JOINTS)
        leg_state[name] = (joint_ids, joint_names)
    print(f"[INFO] Driven: {DRIVEN_JOINTS}")

    while simulation_app.is_running():
        if count % 1000 == 0:
            for leg in legs.values():
                joint_pos = leg.data.default_joint_pos.clone()
                joint_vel = leg.data.default_joint_vel.clone()
                leg.write_joint_state_to_sim(joint_pos, joint_vel)
                leg.reset()
            t = 0.0
            print("[INFO] Reset all 4 legs.")

        for name, leg in legs.items():
            joint_ids, joint_names = leg_state[name]
            body_phase = leg_phases[name]
            targets = torch.zeros((leg.num_instances, len(joint_ids)), device=leg.device)
            for col, jname in enumerate(joint_names):
                amp, freq, joint_phase = JOINT_PARAMS[jname]
                targets[:, col] = amp * math.sin(2.0 * math.pi * freq * t + joint_phase + body_phase)
            leg.set_joint_position_target(targets, joint_ids=joint_ids)
            leg.write_data_to_sim()

        sim.step()
        for leg in legs.values():
            leg.update(sim_dt)
        t += sim_dt
        count += 1


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([1.5, 1.5, 1.0], [0.0, 0.0, 0.4])

    legs, leg_phases = design_scene(args_cli.usd)

    sim.reset()
    print("[INFO] Setup complete.")
    run(sim, legs, leg_phases)


if __name__ == "__main__":
    main()
    simulation_app.close()
