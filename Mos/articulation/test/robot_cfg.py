import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/sz/code/rl/IsaacLab-Mos/Mos/assets/mos/mos.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={
            "hip_joint": 0.0,
            "font_gear": 0.0,
            "small_shaft": 0.0,
            "shank_joint": 0.0,
            "rear_gear": 0.0,
            "large_shaft_thigh": 0.0,
            "shank": 0.0,
        },
    ),
    actuators={
        # TODO: 按实际需要分组，调整 stiffness/damping/effort_limit_sim
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=['hip_joint', 'font_gear', 'small_shaft', 'shank_joint', 'rear_gear', 'large_shaft_thigh', 'shank'],
            effort_limit_sim=23.6,
            stiffness=80.0,
            damping=4.0,
        ),
    },
)
