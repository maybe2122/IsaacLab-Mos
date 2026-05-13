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
    choices=["trot", "bound", "gallop", "dance"],
    help="Gait pattern. trot = slow walk (diagonal pairs); bound = run (front pair + rear pair); "
         "gallop = fast run (asymmetric 4-beat); dance = in-place choreography (bob + sway + paw wave).",
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
#   phase:      0..1 cycle offset per leg (0 = lead, 0.5 = anti-phase)
#   freq:       step cycles per second
#   duty:       fraction of cycle spent in stance (foot on ground). >0.5 walk-like,
#               <0.5 introduces an aerial phase (true run).
#   thigh_amp:  half stride angle (rad). With our sign convention, negative drives
#               the dog forward — stance sweep goes from +thigh_amp to -thigh_amp,
#               which is "foot ahead → foot behind" relative to the hip.
#   shank_lift: knee flexion during swing (rad, negative = knee bends, foot up).
#   hip_offset: static abduction (rad, outward positive); widens stance.
#   hip_lift:   extra outward abduction synced to the swing-phase lift signal,
#               so the foot clears the ground laterally as well as vertically.
GAIT_PRESETS = {
    "trot": {
        "phase": {"fl": 0.0, "rr": 0.0, "fr": 0.5, "rl": 0.5},
        "freq": 1.6,
        "duty": 0.55,
        "thigh_amp": -0.22,
        "shank_lift": -0.35,
        "hip_offset": 0.06,
        "hip_lift": 0.08,
    },
    # Bound: front pair together, rear pair together, 180° apart. Brief aerial phase.
    "bound": {
        "phase": {"fl": 0.0, "fr": 0.0, "rl": 0.5, "rr": 0.5},
        "freq": 2.4,
        "duty": 0.42,
        "thigh_amp": -0.32,
        "shank_lift": -0.50,
        "hip_offset": 0.07,
        "hip_lift": 0.10,
    },
    # Transverse gallop: rear pair leads, front pair lands with a small lateral lag.
    # Sequence in one cycle: RL -> RR -> FL -> FR (typical right-lead gallop).
    "gallop": {
        "phase": {"rl": 0.00, "rr": 0.12, "fl": 0.55, "fr": 0.67},
        "freq": 3.0,
        "duty": 0.38,
        "thigh_amp": -0.38,
        "shank_lift": -0.55,
        "hip_offset": 0.08,
        "hip_lift": 0.12,
    },
}

THIGH_OFFSET = 0.0
SHANK_OFFSET = 0.0

INIT_BASE_HEIGHT = 0.30

STATIC_BASE = False
STATIC_BASE_HEIGHT = 0.20


def make_cfg(usd_path: str) -> ArticulationCfg:
    gait_cfg = GAIT_PRESETS.get(args_cli.gait, {})
    spawn_hip_offset = gait_cfg.get("hip_offset", 0.05)
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
                effort_limit_sim=40.0,
                velocity_limit_sim=12.0,
                stiffness=100.0,
                damping=4.5,
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
    # Crucial: combine_mode="max" — PhysX defaults to averaging the two contacting
    # materials' friction, so even with ground=1.2 the effective μ collapses to
    # ~0.85 against the robot's default 0.5. "max" makes the ground's value win.
    ground_cfg = sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=3.0,
            dynamic_friction=1.3,
            restitution=0.0,
            friction_combine_mode="max",
            restitution_combine_mode="min",
        ),
    )
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    sim_utils.create_prim("/World/Origin1", "Xform", translation=(0.0, 0.0, 0.0))

    robot_cfg = make_cfg(usd_path)
    robot_cfg.prim_path = "/World/Origin1/Robot"
    return Articulation(cfg=robot_cfg)


def phase_curves(phi: float, duty: float) -> tuple[float, float]:
    """Two-phase gait primitive.

    Returns (sweep, lift) for normalized phase phi in [0, 1):
      sweep ∈ [-1, +1] — fore/aft foot position relative to hip.
        Stance (phi < duty): half-cosine from +1 → -1. Cosine (vs. linear) gives
        zero sweep-velocity at touchdown AND liftoff, so the foot doesn't punch
        the ground sideways at impact — the main source of micro-slip.
        Swing  (phi >= duty): mirrored half-cosine from -1 → +1.
      lift ∈ [0, 1] — vertical foot clearance above the ground line.
        Zero throughout stance; a single sin(π·s) arc during swing.

    Both segments end with zero derivative, so velocity is continuous at every
    phase boundary — no impulses, no jitter.
    """
    if phi < duty:
        s = phi / duty
        sweep = math.cos(math.pi * s)
        lift = 0.0
    else:
        s = (phi - duty) / (1.0 - duty)
        sweep = -math.cos(math.pi * s)
        lift = math.sin(math.pi * s)
    return sweep, lift


def build_targets(t: float, joint_names: list[str], device, n_env: int, gait: dict) -> torch.Tensor:
    targets = torch.zeros((n_env, len(joint_names)), device=device)
    name_to_idx = {n: i for i, n in enumerate(joint_names)}
    freq = gait["freq"]
    duty = gait["duty"]
    thigh_amp = gait["thigh_amp"]
    shank_lift = gait["shank_lift"]
    hip_offset = gait.get("hip_offset", 0.0)
    hip_lift = gait.get("hip_lift", 0.0)
    for lr in LEGS:
        s = LEG_AXIS_SIGN[lr]
        phi = (freq * t + gait["phase"][lr]) % 1.0
        sweep, lift = phase_curves(phi, duty)

        # thigh sweeps over its full ±amp range during stance (foot stays on the
        # ground), then returns over swing. shank flexes only during swing,
        # giving a clean step rather than a continuous circle.
        thigh_cmd = THIGH_OFFSET + thigh_amp * sweep
        shank_cmd = SHANK_OFFSET + shank_lift * lift
        # hip: static splay for a stable stance + a swing-synced outward kick
        # so feet clear the body without dragging.
        hip_cmd = hip_offset + hip_lift * lift

        for seg, cmd in (("hip", hip_cmd), ("thigh", thigh_cmd), ("shank", shank_cmd)):
            jname = f"{lr}_{seg}"
            idx = name_to_idx.get(jname)
            if idx is None:
                continue
            targets[:, idx] = s[seg] * cmd
    return targets


# === Dance choreography ===========================================================
# A short in-place dance loop. Three musical layers, all synced to a tempo:
#   1. Bob       — every leg flexes its shank in unison; with feet planted, the
#                  body squats up and down on the beat.
#   2. Sway      — body weight shifts left↔right via asymmetric hip rotation
#                  (left legs and right legs rotate opposite world directions).
#                  Half-tempo so the sway feels lazier than the bob.
#   3. Paw wave  — every 4 beats one front leg lifts, "waves" with a quick
#                  shank wiggle, and lowers. FL and FR take turns.
#
# Pure pose targets — no foot trajectory, no IK. Tuned amplitudes keep the
# planted three legs stable while one paw is in the air.
DANCE_TEMPO_HZ = 1.5            # ≈ 90 BPM
DANCE_BOB_AMP = -0.16           # shank flex (knee bend → body drops)
DANCE_SWAY_AMP = 0.18           # hip abduction asymmetry → lateral lean
DANCE_HIP_BASE = 0.06           # static splay
DANCE_PAW_THIGH_AMP = -0.65     # forward lift of the waving leg's thigh
DANCE_PAW_SHANK_AMP = -0.55     # extra knee flex on the lifted leg
DANCE_WAVE_WIGGLE_HZ = 4.0      # paw wiggle frequency while raised
DANCE_WAVE_WIGGLE_AMP = 0.20    # extra shank wiggle during the wave

# Sway polarity: which way each leg rotates so the body leans together.
# Left legs need opposite sign from right legs after LEG_AXIS_SIGN cancels
# out the URDF axis differences (hip axis sign is +1 for FL/FR, -1 for RL/RR).
SWAY_POL = {"fl": -1.0, "rl": +1.0, "fr": +1.0, "rr": -1.0}


def build_dance_targets(t: float, joint_names: list[str], device, n_env: int) -> torch.Tensor:
    targets = torch.zeros((n_env, len(joint_names)), device=device)
    name_to_idx = {n: i for i, n in enumerate(joint_names)}

    beat = 2.0 * math.pi * DANCE_TEMPO_HZ * t

    # Bob: 0..1, peaks every beat.
    bob = 0.5 * (1.0 - math.cos(2.0 * beat))
    # Sway: -1..1, half-tempo so the body leans for a full beat each side.
    sway = math.sin(beat)

    # 4-beat paw-wave cycle: beats 0-1 FL waves, 1-2 rest, 2-3 FR waves, 3-4 rest.
    cycle_len = 4.0 / DANCE_TEMPO_HZ
    beat_len = 1.0 / DANCE_TEMPO_HZ
    t_in_cycle = t % cycle_len

    def _wave_envelope(start: float, dur: float) -> float:
        if not (start <= t_in_cycle < start + dur):
            return 0.0
        u = (t_in_cycle - start) / dur                 # 0..1
        bell = math.sin(math.pi * u)                   # smooth lift in/out
        wiggle = 0.5 + 0.5 * math.sin(2.0 * math.pi * DANCE_WAVE_WIGGLE_HZ * (t - start))
        return bell * wiggle

    fl_wave = _wave_envelope(0.0 * beat_len, 1.0 * beat_len)
    fr_wave = _wave_envelope(2.0 * beat_len, 1.0 * beat_len)

    for lr in LEGS:
        s = LEG_AXIS_SIGN[lr]

        hip_cmd = DANCE_HIP_BASE + DANCE_SWAY_AMP * SWAY_POL[lr] * sway
        shank_cmd = DANCE_BOB_AMP * bob
        thigh_cmd = 0.0

        # Overlay the wave on the lifted front leg. Replace the bob on that leg
        # so the raised paw doesn't fight a downward squat command.
        if lr == "fl" and fl_wave > 0.0:
            thigh_cmd = DANCE_PAW_THIGH_AMP * fl_wave
            shank_cmd = DANCE_PAW_SHANK_AMP * fl_wave \
                + DANCE_WAVE_WIGGLE_AMP * (fl_wave - 0.5) * 2.0
        elif lr == "fr" and fr_wave > 0.0:
            thigh_cmd = DANCE_PAW_THIGH_AMP * fr_wave
            shank_cmd = DANCE_PAW_SHANK_AMP * fr_wave \
                + DANCE_WAVE_WIGGLE_AMP * (fr_wave - 0.5) * 2.0

        for seg, cmd in (("hip", hip_cmd), ("thigh", thigh_cmd), ("shank", shank_cmd)):
            jname = f"{lr}_{seg}"
            idx = name_to_idx.get(jname)
            if idx is None:
                continue
            targets[:, idx] = s[seg] * cmd
    return targets


def run(sim: SimulationContext, robot: Articulation, gait):
    sim_dt = sim.get_physics_dt()
    t = 0.0
    count = 0
    is_dance = gait is None

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

        if is_dance:
            targets_full = build_dance_targets(t, robot.data.joint_names, robot.device, robot.num_instances)
        else:
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
    if args_cli.gait == "dance":
        print(f"[INFO] Gait = dance  tempo={DANCE_TEMPO_HZ} Hz  bob={DANCE_BOB_AMP}  "
              f"sway={DANCE_SWAY_AMP}  paw_thigh={DANCE_PAW_THIGH_AMP}")
        gait = None
    else:
        gait = GAIT_PRESETS[args_cli.gait]
        print(f"[INFO] Gait = {args_cli.gait}  freq={gait['freq']} Hz  duty={gait['duty']}  "
              f"thigh_amp={gait['thigh_amp']}  shank_lift={gait['shank_lift']}  "
              f"hip_offset={gait['hip_offset']}  hip_lift={gait['hip_lift']}")
    print("[INFO] Setup complete.")
    run(sim, robot, gait)


if __name__ == "__main__":
    main()
    simulation_app.close()
