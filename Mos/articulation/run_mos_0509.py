"""在 Isaac Sim 中加载 mos_0509.usd 并让三个驱动关节做正弦摆动。

Usage:
    ./isaaclab.sh -p Mos/articulation/run_mos_0509.py
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn and animate mos_0509.")
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
# Passive joints in the closed loop — must NOT be position-driven, otherwise
# the URDF→USD default drive (stiffness≈1.745) locks them at 0.
# thigh_gear and shank_gear are driven via PhysxMimicJointAPI from rear_gear /
# font_gear respectively (gearing=+1 -> q_target = -q_ref, i.e. external mesh).
PASSIVE_JOINTS = ["shank_middle", "shank_link", "thigh_gear", "shank_gear"]

# shank_loop closure pin local positions (from GUI calibration, mos_0509.usd).
LOOP_BODY0 = "shank"
LOOP_BODY1 = "shank_link"
LOOP_POS0 = (-0.06813, 0.0, 0.1038)        # in shank body frame
LOOP_POS1 = (-0.0851, 0.00385, -0.13561)   # in shank_link body frame; +Y compensates URDF chain Y mismatch


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
                solver_position_iteration_count=64,
                solver_velocity_iteration_count=4,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.6),
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

    sim_utils.create_prim("/World/Origin1", "Xform", translation=(0.0, 0.0, 0.0))

    robot_cfg = make_cfg(usd_path)
    robot_cfg.prim_path = "/World/Origin1/Robot"
    return {"robot": Articulation(cfg=robot_cfg)}


def quat_apply(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate vector v by quaternion q (w,x,y,z). Both broadcast on leading dims."""
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    vx, vy, vz = v[..., 0], v[..., 1], v[..., 2]
    # t = 2 * (q.xyz x v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v + w*t + q.xyz x t
    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return torch.stack([rx, ry, rz], dim=-1)


def run(sim: SimulationContext, robot: Articulation):
    sim_dt = sim.get_physics_dt()
    t = 0.0
    count = 0

    joint_ids, joint_names = robot.find_joints(DRIVEN_JOINTS)
    print(f"[INFO] Driving joints: {joint_names} -> indices {joint_ids}")
    print(f"[INFO] All joints in articulation: {robot.data.joint_names}")
    print(f"[INFO] All bodies in articulation: {robot.data.body_names}")

    body0_ids, _ = robot.find_bodies([LOOP_BODY0])
    body1_ids, _ = robot.find_bodies([LOOP_BODY1])
    body0_idx = body0_ids[0]
    body1_idx = body1_ids[0]
    print(f"[INFO] Loop closure: body0={LOOP_BODY0}(idx={body0_idx}) body1={LOOP_BODY1}(idx={body1_idx})")

    pos0_local = torch.tensor(LOOP_POS0, device=robot.device).unsqueeze(0).expand(robot.num_instances, 3)
    pos1_local = torch.tensor(LOOP_POS1, device=robot.device).unsqueeze(0).expand(robot.num_instances, 3)

    # Mimic relation: q_target + gearing*q_ref + offset = 0  =>  q_target = -q_ref.
    # So drive font_gear / rear_gear with the negation of the previous
    # shank_gear / thigh_gear targets to preserve the gait.
    name_to_params = {
        "hip_joint": (0.25, 0.2, 0.0),
        "rear_gear": (-0.4, 0.15, math.pi / 4),  # mirrors old thigh_gear
        "font_gear": (0.4, -0.15, math.pi / 2),  # mirrors old shank_gear
    }

    while simulation_app.is_running():
        if count % 1000 == 0:
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            t = 0.0
            print("[INFO] Reset.")

        targets = torch.zeros((robot.num_instances, len(joint_ids)), device=robot.device)
        for col, name in enumerate(joint_names):
            amp, freq, phase = name_to_params[name]
            targets[:, col] = amp * math.sin(2.0 * math.pi * freq * t + phase)

        robot.set_joint_position_target(targets, joint_ids=joint_ids)
        robot.write_data_to_sim()

        sim.step()
        robot.update(sim_dt)

        # Diagnose loop closure error: world distance between pos0 (in shank) and pos1 (in shank_link).
        if count % 60 == 0:
            body_pos_w = robot.data.body_pos_w  # (N, B, 3)
            body_quat_w = robot.data.body_quat_w  # (N, B, 4) wxyz
            p0_w = body_pos_w[:, body0_idx] + quat_apply(body_quat_w[:, body0_idx], pos0_local)
            p1_w = body_pos_w[:, body1_idx] + quat_apply(body_quat_w[:, body1_idx], pos1_local)
            delta = (p1_w - p0_w)[0]
            err_mm = float(torch.linalg.norm(delta)) * 1000.0
            print(
                f"[LOOP] t={t:6.2f}s  shank_anchor_w={p0_w[0].cpu().numpy()}  "
                f"link_anchor_w={p1_w[0].cpu().numpy()}  |delta|={err_mm:.2f} mm"
            )

        t += sim_dt
        count += 1


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([1.5, 1.5, 1.0], [0.0, 0.0, 0.4])

    entities = design_scene(args_cli.usd)

    sim.reset()
    print("[INFO] Setup complete.")
    run(sim, entities["robot"])


if __name__ == "__main__":
    main()
    simulation_app.close()
