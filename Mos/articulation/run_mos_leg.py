"""在 Isaac Sim 中加载 mos_leg_51.usd 并让三个驱动关节做正弦摆动。

Usage:
    ./isaaclab.sh -p Mos/articulation/run_mos_leg.py
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn and animate mos_leg_51.")
parser.add_argument(
    "--usd",
    type=str,
    default=os.path.join(os.path.dirname(__file__), "..", "assets", "mos_leg_51", "mos_leg_51.usd"),
    help="Path to the converted leg USD.",
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


# Drivable joints (effort > 0 in the URDF)
DRIVEN_JOINTS = ["hip_joint", "thigh_gear", "shank_gear"]


def make_cfg(usd_path: str) -> ArticulationCfg:
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
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.6),
            joint_pos={n: 0.0 for n in DRIVEN_JOINTS},
        ),
        actuators={
            "all": ImplicitActuatorCfg(
                joint_names_expr=DRIVEN_JOINTS,
                effort_limit_sim=30.0,
                velocity_limit_sim=3.0,
                stiffness=80.0,
                damping=4.0,
            ),
        },
    )


def design_scene(usd_path: str):
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    sim_utils.create_prim("/World/Origin1", "Xform", translation=(0.0, 0.0, 0.0))

    leg_cfg = make_cfg(usd_path)
    leg_cfg.prim_path = "/World/Origin1/Robot"
    return {"leg": Articulation(cfg=leg_cfg)}


def run(sim: SimulationContext, leg: Articulation):
    sim_dt = sim.get_physics_dt()
    t = 0.0
    count = 0

    # Get joint indices for the driven joints (within the articulation)
    joint_ids, joint_names = leg.find_joints(DRIVEN_JOINTS)
    print(f"[INFO] Driving joints: {joint_names} -> indices {joint_ids}")
    print(f"[INFO] All joints in articulation: {leg.data.joint_names}")

    # Per-joint sinusoid parameters: (amplitude, frequency Hz, phase)
    # Amplitudes stay within URDF limits: hip ±0.5, thigh/shank ±1.0
    name_to_params = {
        "hip_joint": (0.25, 0.2, 0.0),
        "thigh_gear": (0.4, 0.15, math.pi / 4),
        "shank_gear": (0.5, 0.15, math.pi / 2),
    }

    while simulation_app.is_running():
        if count % 1000 == 0:
            joint_pos = leg.data.default_joint_pos.clone()
            joint_vel = leg.data.default_joint_vel.clone()
            leg.write_joint_state_to_sim(joint_pos, joint_vel)
            leg.reset()
            t = 0.0
            print("[INFO] Reset.")

        # Build target tensor only for driven joints
        targets = torch.zeros((leg.num_instances, len(joint_ids)), device=leg.device)
        for col, name in enumerate(joint_names):
            amp, freq, phase = name_to_params[name]
            targets[:, col] = amp * math.sin(2.0 * math.pi * freq * t + phase)

        leg.set_joint_position_target(targets, joint_ids=joint_ids)
        leg.write_data_to_sim()

        sim.step()
        leg.update(sim_dt)
        t += sim_dt
        count += 1


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([1.5, 1.5, 1.0], [0.0, 0.0, 0.4])

    entities = design_scene(args_cli.usd)

    sim.reset()
    print("[INFO] Setup complete.")
    run(sim, entities["leg"])


if __name__ == "__main__":
    main()
    simulation_app.close()
