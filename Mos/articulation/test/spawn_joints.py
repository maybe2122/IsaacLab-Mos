
"""
离线扫描 USD 关节并生成 ArticulationCfg 代码文件。
使用方式：在安装了 pxr (usd-core) 的普通 Python 环境下运行。
    pip install usd-core
    python gen_cfg.py
"""
from pxr import Usd, UsdPhysics
import os

# === 1. 替换为本地 USD 路径 ===
usd_path = f"/home/sz/code/rl/IsaacLab-Mos/Mos/assets/mos/mos.usd"
output_path = "robot_cfg.py"

stage = Usd.Stage.Open(usd_path)
if not stage:
    raise RuntimeError(f"无法打开 USD 文件: {usd_path}")

# === 2. 扫描关节（按类型过滤） ===
joint_names = []
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
        joint_names.append(prim.GetName())

print(f"检测到 {len(joint_names)} 个关节: {joint_names}")

# === 3. 生成 Python 配置代码并写入文件 ===
joint_pos_lines = "\n".join(
    f'            "{name}": 0.0,' for name in joint_names
)
joint_names_expr = str(joint_names)

code = f'''\
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

ROBOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="{usd_path}",
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
        joint_pos={{
{joint_pos_lines}
        }},
    ),
    actuators={{
        # TODO: 按实际需要分组，调整 stiffness/damping/effort_limit_sim
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr={joint_names_expr},
            effort_limit_sim=100.0,
            stiffness=80.0,
            damping=4.0,
        ),
    }},
)
'''

with open(output_path, "w", encoding="utf-8") as f:
    f.write(code)

print(f"配置文件已写入: {output_path}")
