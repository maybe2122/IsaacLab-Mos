"""在 Isaac Sim 中加载 mos2026_2.usd 并用 trot 步态让它在地上走。

驱动策略与 run_mos2026.py 相同：每条腿驱动 hip/thigh/shank 三个关节（共 12 个），
其余齿轮 / 闭链关节做 passive。

用法::

    ./isaaclab.sh -p Mos/articulation/run_mos2026_2.py
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn and walk/run mos2026_2.")
parser.add_argument(
    "--usd",
    type=str,
    default=os.path.join(os.path.dirname(__file__), "..", "assets", "mos2026_2.usd"),
    help="Path to the mos2026_2 USD.",
)
parser.add_argument(
    "--gait",
    type=str,
    default="trot",
    choices=["trot", "bound", "gallop"],
    help="Gait pattern. trot = slow walk (diagonal pairs); bound = run (front pair + rear pair); "
         "gallop = fast run (asymmetric 4-beat).",
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


LEGS = ["fl", "fr", "rl", "rr"]
DRIVEN_JOINTS = [f"{lr}_{seg}" for lr in LEGS for seg in ("hip", "thigh", "shank")]

PASSIVE_PATTERNS = [
    r".*_shank_motor(_gear)?",
    r".*_thigh_motor(_gear)?",
    r".*_shank_link(_a)?",
    r".*_shank_link_b",
]

LEG_AXIS_SIGN = {
    "fl": {"hip": +1.0, "thigh": +1.0, "shank": +1.0},
    "fr": {"hip": +1.0, "thigh": -1.0, "shank": -1.0},
    "rl": {"hip": -1.0, "thigh": +1.0, "shank": +1.0},
    "rr": {"hip": -1.0, "thigh": -1.0, "shank": -1.0},
}

# Per-gait timing and amplitude.
#   phase: 0..1 cycle offset per leg (0 = lead, 0.5 = anti-phase)
#   freq:  step cycles per second
#   thigh_amp / shank_amp: swing amplitude (rad)
#   hip_offset: static abduction (rad, outward positive); widens stance for stability
#   hip_lift:   extra abduction during swing only (rad, outward positive); lifts foot
#               clear of ground during the swing half-cycle so it doesn't drag.
GAIT_PRESETS = {
    "trot": {
        "phase": {"fl": 0.0, "rr": 0.0, "fr": 0.5, "rl": 0.5},
        "freq": 1.0,
        "thigh_amp": -0.15,
        "shank_amp": 0.20,
        "hip_offset": 0.08,
        "hip_lift": 0.10,
    },
    # Bound: front pair together, rear pair together, 180° apart. Higher freq + amp -> run.
    "bound": {
        "phase": {"fl": 0.0, "fr": 0.0, "rl": 0.5, "rr": 0.5},
        "freq": 2.2,
        "thigh_amp": -0.30,
        "shank_amp": 0.35,
        "hip_offset": 0.10,
        "hip_lift": 0.15,
    },
    # Transverse gallop: rear pair leads, front pair lands with a small lateral lag.
    # Sequence in one cycle: RL -> RR -> FL -> FR (typical right-lead gallop).
    "gallop": {
        "phase": {"rl": 0.00, "rr": 0.10, "fl": 0.55, "fr": 0.65},
        "freq": 2.8,
        "thigh_amp": -0.35,
        "shank_amp": 0.40,
        "hip_offset": 0.10,
        "hip_lift": 0.18,
    },
}

GAIT_FREQ = 1.0
THIGH_OFFSET = 0.0
SHANK_OFFSET = 0.0
THIGH_AMP = -0.15
SHANK_AMP = 0.20
HIP_AMP = 0.0

INIT_BASE_HEIGHT = 0.30

STATIC_BASE = False
STATIC_BASE_HEIGHT = 0.20


def make_cfg(usd_path: str) -> ArticulationCfg:
    spawn_hip_offset = GAIT_PRESETS[args_cli.gait].get("hip_offset", 0.0)
    init_joint_pos: dict[str, float] = {}
    for lr in LEGS:
        s = LEG_AXIS_SIGN[lr]
        init_joint_pos[f"{lr}_hip"] = s["hip"] * spawn_hip_offset
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
                max_depenetration_velocity=1.0,
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
                damping=0.5,
            ),
        },
    )


def design_scene(usd_path: str):
    # Higher friction ground to keep the feet from sliding during stance push-off.
    ground_cfg = sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.2,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    sim_utils.create_prim("/World/Origin1", "Xform", translation=(0.0, 0.0, 0.0))

    robot_cfg = make_cfg(usd_path)
    robot_cfg.prim_path = "/World/Origin1/Robot"
    return Articulation(cfg=robot_cfg)


def build_targets(t: float, joint_names: list[str], device, n_env: int, gait: dict) -> torch.Tensor:
    targets = torch.zeros((n_env, len(joint_names)), device=device)
    name_to_idx = {n: i for i, n in enumerate(joint_names)}
    freq = gait["freq"]
    thigh_amp = gait["thigh_amp"]
    shank_amp = gait["shank_amp"]
    hip_offset = gait.get("hip_offset", 0.0)
    hip_lift = gait.get("hip_lift", 0.0)
    for lr in LEGS:
        s = LEG_AXIS_SIGN[lr]
        phase = 2.0 * math.pi * gait["phase"][lr]
        omega = 2.0 * math.pi * freq * t + phase
        thigh_cmd = THIGH_OFFSET + thigh_amp * math.sin(omega)
        shank_cmd = SHANK_OFFSET - shank_amp * math.cos(omega)
        # Swing window: shank is bent (foot up) when cos(omega) > 0. Use the positive
        # half of cos(omega) as a swing gate so hip abducts outward only when the
        # foot is in the air, never during stance push-off.
        swing_gate = max(0.0, math.cos(omega))
        hip_cmd = hip_offset + hip_lift * swing_gate

        for seg, cmd in (("hip", hip_cmd), ("thigh", thigh_cmd), ("shank", shank_cmd)):
            jname = f"{lr}_{seg}"
            idx = name_to_idx.get(jname)
            if idx is None:
                continue
            targets[:, idx] = s[seg] * cmd
    return targets


def run(sim: SimulationContext, robot: Articulation, gait: dict):
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

        targets_full = build_targets(t, robot.data.joint_names, robot.device, robot.num_instances, gait)
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
    gait = GAIT_PRESETS[args_cli.gait]
    print(f"[INFO] Gait = {args_cli.gait}  freq={gait['freq']} Hz  "
          f"thigh_amp={gait['thigh_amp']}  shank_amp={gait['shank_amp']}")
    print("[INFO] Setup complete.")
    run(sim, robot, gait)


if __name__ == "__main__":
    main()
    simulation_app.close()
