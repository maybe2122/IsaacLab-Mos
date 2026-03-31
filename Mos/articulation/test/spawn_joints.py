# -*- coding: utf-8 -*-
"""
Auto-generate articulation configuration for a robot USD in Isaac Lab.

Instructions:
1. 将 `usd_path` 替换为你自己的机器人 USD 文件路径。
2. 运行此脚本，它会读取 USD 中所有关节，并生成初始 joint_pos 和 actuator 配置。
3. 根据需要修改 stiffness/damping/effort_limit。
"""

import omni.usd
from pxr import Usd, UsdGeom, PhysxSchema
from isaaclab.sim import UsdFileCfg, RigidBodyPropertiesCfg, ArticulationRootPropertiesCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

# === 1. 设置你的 USD 路径 ===
usd_path = f"{ISAACLAB_NUCLEUS_DIR}/path/to/your_robot.usd"

# 打开 USD stage
stage = Usd.Stage.Open(usd_path)
if not stage:
    raise RuntimeError(f"无法打开 USD 文件: {usd_path}")

# === 2. 扫描关节 ===
joint_paths = []

def collect_joints(prim):
    from pxr import PhysxSchema
    if PhysxSchema.PhysxArticulationJointAPI.HasAPI(prim):
        joint_paths.append(prim.GetName())
    for child in prim.GetChildren():
        collect_joints(child)

collect_joints(stage.GetPseudoRoot())

print(f"检测到 {len(joint_paths)} 个关节: {joint_paths}")

# === 3. 自动生成 joint_pos 和 actuator 配置 ===
joint_pos = {name: 0.0 for name in joint_paths}  # 初始角度/位置都为 0
actuators = {
    f"{name}_actuator": ImplicitActuatorCfg(
        joint_names_expr=[name],
        effort_limit_sim=100.0,  # 默认最大力矩/力
        stiffness=0.0,           # 默认刚度
        damping=1.0,             # 默认阻尼
    )
    for name in joint_paths
}

# === 4. 生成 ArticulationCfg ===
ROBOT_CFG = ArticulationCfg(
    spawn=UsdFileCfg(
        usd_path=usd_path,
        rigid_props=RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),  # 世界初始位置
        joint_pos=joint_pos
    ),
    actuators=actuators
)

print("ArticulationCfg 模板生成完成！")