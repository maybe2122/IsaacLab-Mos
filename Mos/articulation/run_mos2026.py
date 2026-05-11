"""在 Isaac Sim 中加载 mos2026.usd 并用 trot 步态让它在地上走。

简化驱动：每条腿直接驱动 hip/thigh/shank 三个关节（共 12 个），motor_gear、
shank_link、shank_link_b 全部 passive。原 URDF 里的齿轮 mimic 与
shank↔shank_link_b 闭链没有显式约束，仍然作为自由 DoF 存在，仅作视觉。

如果还没有 USD，先转换一次（fix_urdf.py 已经处理 RR 的 fixed joint 和 effort=0
的问题，请使用 mos2026_fixed.urdf）::

    python3 Mos/urdf/mos2026/fix_urdf.py
    ./isaaclab.sh -p scripts/tools/convert_urdf.py \
        Mos/urdf/mos2026/urdf/mos2026_fixed.urdf \
        Mos/assets/mos2026.usd --merge-joints

然后::

    ./isaaclab.sh -p Mos/articulation/run_mos2026.py
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn and trot mos2026.")
parser.add_argument(
    "--usd",
    type=str,
    default=os.path.join(os.path.dirname(__file__), "..", "assets", "mos2026.usd"),
    help="Path to the mos2026 USD.",
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


# Hip / thigh / shank revolute joint per leg. These drive the leg directly,
# bypassing the URDF's gear mechanism (which has no mimic constraints).
LEGS = ["fl", "fr", "rl", "rr"]
DRIVEN_JOINTS = [f"{lr}_{seg}" for lr in LEGS for seg in ("hip", "thigh", "shank")]

# Everything else on the closed-chain side gets zero-stiffness passive drive
# so PhysX doesn't lock it at 0 with the converter's default gains.
# Naming varies by leg (fl_shank_motor_gear vs fr_shank_motor), so we match
# both forms with a regex.
PASSIVE_PATTERNS = [
    r".*_shank_motor(_gear)?",
    r".*_thigh_motor(_gear)?",
    r".*_shank_link(_a)?",
    r".*_shank_link_b",
]

# Per-leg axis signs so the same nominal (thigh, shank) angles produce the
# same world-frame motion across all four legs.
#   hip:   FL/FR axis -X, RL/RR axis +X -> rear legs get -1
#   thigh: left axis -Y, right axis +Y  -> right legs get -1
#   shank: same as thigh
LEG_AXIS_SIGN = {
    "fl": {"hip": +1.0, "thigh": +1.0, "shank": +1.0},
    "fr": {"hip": +1.0, "thigh": -1.0, "shank": -1.0},
    "rl": {"hip": -1.0, "thigh": +1.0, "shank": +1.0},
    "rr": {"hip": -1.0, "thigh": -1.0, "shank": -1.0},
}

# Trot: diagonal pairs (FL,RR) and (FR,RL) are in phase, 180° apart.
LEG_PHASE = {
    "fl": 0.0,
    "rr": 0.0,
    "fr": math.pi,
    "rl": math.pi,
}

# Gait shape in the leg's local convention (axis signs above flip these per leg).
# Why zero offsets work:
#   Each leg's zero-q pose is already a knee-bent V stance — thigh goes
#   forward-down ~61° from vertical, shank then goes backward-down ~61°, and
#   the foot lands ~0.16 m below the base origin. So q=0 on every leg gives
#   the natural standing pose; we just need to spawn the base at that height.
#   (Both -1.07 and +1.07 thigh offsets flip the V open or twist the shank up,
#   neither holds the body. Don't add a thigh offset for the standing config.)
GAIT_FREQ = 1.0                       # Hz; 稳了之后再加速
THIGH_OFFSET = 0.0                    # zero-q is the natural stance
SHANK_OFFSET = 0.0
THIGH_AMP = 0.15                      # thigh 前后摆 ±0.15 rad ≈ ±8.6°
SHANK_AMP = 0.20                      # shank 与 thigh 相差 90° → 抬腿落腿
HIP_AMP = 0.0                          # 不需要外展，trot 走直线

INIT_BASE_HEIGHT = 0.30                # 留充足空中余量，让脚自由下落，避免 spawn 一帧穿模被弹飞

# Debug: set True to fix the base in the air so legs hang freely. Useful for
# eyeballing the natural pose without the body dropping. Disable for walking.
STATIC_BASE = False
STATIC_BASE_HEIGHT = 0.20


def make_cfg(usd_path: str) -> ArticulationCfg:
    init_joint_pos: dict[str, float] = {}
    for lr in LEGS:
        s = LEG_AXIS_SIGN[lr]
        init_joint_pos[f"{lr}_hip"] = 0.0
        init_joint_pos[f"{lr}_thigh"] = s["thigh"] * THIGH_OFFSET
        init_joint_pos[f"{lr}_shank"] = s["shank"] * SHANK_OFFSET

    base_z = STATIC_BASE_HEIGHT if STATIC_BASE else INIT_BASE_HEIGHT
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.abspath(usd_path),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                max_linear_velocity=20.0,
                max_angular_velocity=20.0,
                max_depenetration_velocity=1.0,        # 关键：100 会把哪怕 1mm 穿模都反弹成弹飞
                enable_gyroscopic_forces=True,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=64,
                solver_velocity_iteration_count=4,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
                fix_root_link=STATIC_BASE,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, base_z),
            joint_pos=init_joint_pos,
        ),
        actuators={
            "driven": ImplicitActuatorCfg(
                joint_names_expr=DRIVEN_JOINTS,
                effort_limit_sim=30.0,
                velocity_limit_sim=10.0,
                stiffness=60.0,
                damping=3.0,
            ),
            "passive": ImplicitActuatorCfg(
                joint_names_expr=PASSIVE_PATTERNS,
                effort_limit_sim=0.0,
                velocity_limit_sim=1000.0,
                stiffness=0.0,
                damping=0.5,                 # tame closed-loop flop on landing
            ),
        },
    )


def design_scene(usd_path: str):
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    sim_utils.create_prim("/World/Origin1", "Xform", translation=(0.0, 0.0, 0.0))

    robot_cfg = make_cfg(usd_path)
    robot_cfg.prim_path = "/World/Origin1/Robot"
    return Articulation(cfg=robot_cfg)


def build_targets(t: float, joint_names: list[str], device, n_env: int) -> torch.Tensor:
    """Compute trot joint targets for every joint in the articulation."""
    targets = torch.zeros((n_env, len(joint_names)), device=device)
    name_to_idx = {n: i for i, n in enumerate(joint_names)}
    for lr in LEGS:
        s = LEG_AXIS_SIGN[lr]
        phase = LEG_PHASE[lr]
        omega = 2.0 * math.pi * GAIT_FREQ * t + phase
        thigh_cmd = THIGH_OFFSET + THIGH_AMP * math.sin(omega)
        shank_cmd = SHANK_OFFSET - SHANK_AMP * math.cos(omega)  # phased w/ thigh
        hip_cmd = HIP_AMP * math.sin(omega)

        for seg, cmd in (("hip", hip_cmd), ("thigh", thigh_cmd), ("shank", shank_cmd)):
            jname = f"{lr}_{seg}"
            idx = name_to_idx.get(jname)
            if idx is None:
                continue
            targets[:, idx] = s[seg] * cmd
    return targets


def run(sim: SimulationContext, robot: Articulation):
    sim_dt = sim.get_physics_dt()
    t = 0.0
    count = 0

    print(f"[INFO] Joint names ({len(robot.data.joint_names)}):")
    for i, n in enumerate(robot.data.joint_names):
        print(f"  [{i:2d}] {n}")
    print(f"[INFO] Body names ({len(robot.data.body_names)}):")
    for i, n in enumerate(robot.data.body_names):
        print(f"  [{i:2d}] {n}")

    driven_ids, driven_names = robot.find_joints(DRIVEN_JOINTS)
    print(f"[INFO] Driven joints -> indices {dict(zip(driven_names, driven_ids))}")

    while simulation_app.is_running():
        if count % 2000 == 0:
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            t = 0.0
            print("[INFO] Reset.")

        targets_full = build_targets(t, robot.data.joint_names, robot.device, robot.num_instances)
        robot.set_joint_position_target(targets_full)
        robot.write_data_to_sim()

        sim.step()
        robot.update(sim_dt)

        if count % 120 == 0:
            base_pos = robot.data.root_pos_w[0].cpu().numpy()
            print(f"[STEP] t={t:5.2f}s  base xyz = {base_pos}")

        t += sim_dt
        count += 1


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([1.8, 1.8, 1.0], [0.0, 0.0, 0.3])

    robot = design_scene(args_cli.usd)

    sim.reset()
    print("[INFO] Setup complete.")
    run(sim, robot)


if __name__ == "__main__":
    main()
    simulation_app.close()
