# Isaac Lab 创建训练环境指南

> 适用版本：Isaac Lab v2.x  
> 前置条件：已完成机器人的 `ArticulationCfg` 配置

---

## 目录

1. [整体结构](#1-整体结构)
2. [SceneCfg 场景配置](#2-scenecfg-场景配置)
3. [ObservationCfg 观测配置](#3-observationcfg-观测配置)
4. [ActionCfg 动作配置](#4-actioncfg-动作配置)
5. [RewardCfg 奖励配置](#5-rewardcfg-奖励配置)
6. [TerminationCfg 终止条件配置](#6-terminationcfg-终止条件配置)
7. [EventCfg 事件配置](#7-eventcfg-事件配置)
8. [EnvironmentCfg 汇总配置](#8-environmentcfg-汇总配置)
9. [注册环境](#9-注册环境)
10. [文件结构](#10-文件结构)

---

## 1. 整体结构

Isaac Lab 的 Manager-Based 环境由以下几个 Cfg 组成，最终汇总到一个 `EnvCfg` 里：

```
ManagerBasedRLEnvCfg
├── SceneCfg          # 场景：机器人、地形、传感器
├── ObservationCfg    # 观测：策略网络的输入
├── ActionCfg         # 动作：策略网络的输出
├── RewardCfg         # 奖励：训练目标
├── TerminationCfg    # 终止：重置条件
└── EventCfg          # 事件：重置时做什么（随机化等）
```

---

## 2. SceneCfg 场景配置

定义仿真场景中有哪些物体，包括机器人、地面、光照等。

```python
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.terrains import TerrainImporterCfg
import isaaclab.sim as sim_utils
from isaaclab.utils import configclass

from your_robot_cfg import MOS_CFG  # 你的机器人配置

@configclass
class MosSceneCfg(InteractiveSceneCfg):

    # 地面
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",          # 平地，也可以换成 "generator" 生成复杂地形
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
    )

    # 机器人（引用你的 ArticulationCfg）
    robot: ArticulationCfg = MOS_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"   # 多环境自动展开
    )

    # 光照
    light = sim_utils.DomeLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75)
    )
```

**关键点：**
- `{ENV_REGEX_NS}` 是 Isaac Lab 的多环境占位符，会自动展开为 `/World/envs/env_0`、`/World/envs/env_1` 等
- `num_envs` 和 `env_spacing` 在 `EnvironmentCfg` 里设置

---

## 3. ObservationCfg 观测配置

定义策略网络每一步能看到什么信息。

```python
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg, SceneEntityCfg
from isaaclab.envs import mdp

@configclass
class MosObservationCfg:

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """策略网络的观测（喂给神经网络的输入）"""

        # 根节点线速度（世界系）
        base_lin_vel = ObservationTermCfg(
            func=mdp.base_lin_vel,
            noise=mdp.GaussianNoiseCfg(std=0.1)   # 加噪声提升鲁棒性
        )

        # 根节点角速度
        base_ang_vel = ObservationTermCfg(
            func=mdp.base_ang_vel,
            noise=mdp.GaussianNoiseCfg(std=0.1)
        )

        # 重力投影方向（反映机器人的倾斜状态）
        projected_gravity = ObservationTermCfg(
            func=mdp.projected_gravity,
            noise=mdp.GaussianNoiseCfg(std=0.05)
        )

        # 速度指令（目标前进速度/转向速度）
        velocity_commands = ObservationTermCfg(
            func=mdp.generated_commands,
            params={"command_name": "base_velocity"}
        )

        # 关节位置
        joint_pos = ObservationTermCfg(
            func=mdp.joint_pos_rel,             # 相对于默认位置的偏差
            noise=mdp.GaussianNoiseCfg(std=0.01)
        )

        # 关节速度
        joint_vel = ObservationTermCfg(
            func=mdp.joint_vel_rel,
            noise=mdp.GaussianNoiseCfg(std=0.05)
        )

        # 上一步动作（历史信息）
        actions = ObservationTermCfg(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True    # 训练时加噪声
            self.concatenate_terms = True    # 拼成一个向量

    policy: PolicyCfg = PolicyCfg()
```

**关键点：**
- 观测维度 = 所有 term 的维度之和，这个数值要和神经网络输入维度一致
- `noise` 是领域随机化的一部分，让策略对传感器噪声鲁棒
- 可以额外定义 `CriticCfg` 给 Critic 网络更多特权信息（如真实速度）

---

## 4. ActionCfg 动作配置

定义策略网络每一步输出什么，以及如何作用到机器人。

```python
from isaaclab.managers import ActionTermCfg
from isaaclab.envs import mdp

@configclass
class MosActionCfg:

    # 关节位置控制（最常用）
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],          # 控制所有关节
        scale=0.5,                   # 动作缩放，网络输出 * scale = 实际目标角度偏差
        use_default_offset=True      # 以 init_state 中的 joint_pos 为零点
    )
```

**其他动作类型：**

```python
# 关节力矩控制
joint_effort = mdp.JointEffortActionCfg(
    asset_name="robot",
    joint_names=[".*"],
    scale=50.0,
)

# 关节速度控制
joint_vel = mdp.JointVelocityActionCfg(
    asset_name="robot",
    joint_names=[".*"],
    scale=1.0,
)
```

**关键点：**
- 动作维度 = 被控制的关节数量，需要和神经网络输出维度一致
- `scale` 很重要，太大会导致动作过激，太小策略学不到有效控制

---

## 5. RewardCfg 奖励配置

定义训练目标，是影响训练效果最大的部分。

```python
from isaaclab.managers import RewardTermCfg, SceneEntityCfg
from isaaclab.envs import mdp

@configclass
class MosRewardCfg:

    # ===== 正奖励（鼓励的行为）=====

    # 跟踪线速度指令（核心奖励）
    track_lin_vel_xy_exp = RewardTermCfg(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.25}
    )

    # 跟踪角速度指令
    track_ang_vel_z_exp = RewardTermCfg(
        func=mdp.track_ang_vel_z_exp,
        weight=0.5,
        params={"command_name": "base_velocity", "std": 0.25}
    )

    # ===== 负奖励/惩罚（抑制的行为）=====

    # 惩罚 Z 轴线速度（不希望机器人上下跳动）
    lin_vel_z_l2 = RewardTermCfg(
        func=mdp.lin_vel_z_l2,
        weight=-2.0
    )

    # 惩罚关节加速度（运动平滑性）
    joint_acc_l2 = RewardTermCfg(
        func=mdp.joint_acc_l2,
        weight=-2.5e-7
    )

    # 惩罚动作变化率（相邻帧动作差异，抑制抖动）
    action_rate_l2 = RewardTermCfg(
        func=mdp.action_rate_l2,
        weight=-0.01
    )

    # 惩罚关节力矩（节省能耗）
    joint_torques_l2 = RewardTermCfg(
        func=mdp.joint_torques_l2,
        weight=-1.5e-7
    )

    # 惩罚倒地（机器人本体接触地面）
    flat_orientation_l2 = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=-5.0
    )
```

**关键点：**
- `weight` 的正负决定鼓励还是惩罚，大小决定该目标的优先级
- 奖励设计是 RL 调参中最耗时的部分，建议从简单奖励开始
- 可以用 `curriculum` 让奖励权重随训练进度变化

---

## 6. TerminationCfg 终止条件配置

定义什么情况下触发 episode 重置。

```python
from isaaclab.managers import TerminationTermCfg, SceneEntityCfg
from isaaclab.envs import mdp

@configclass
class MosTerminationCfg:

    # 超时终止（每个 episode 最长步数）
    time_out = TerminationTermCfg(
        func=mdp.time_out,
        time_out=True           # 标记为超时，不计入失败
    )

    # 机器人本体接触地面（倒地）
    base_contact = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names="base"   # 监测 base link 的接触
            ),
            "threshold": 1.0        # 接触力超过 1N 就终止
        }
    )
```

---

## 7. EventCfg 事件配置

定义 reset 时的随机化操作，提升策略泛化能力（Domain Randomization）。

```python
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.envs import mdp

@configclass
class MosEventCfg:

    # 重置时随机化机器人根节点位置和朝向
    reset_base = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "yaw": (-3.14, 3.14)
            },
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
            },
        }
    )

    # 重置时随机化关节状态
    reset_robot_joints = EventTermCfg(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        }
    )

    # 训练过程中随机推一把机器人（提升抗干扰能力）
    push_robot = EventTermCfg(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),   # 每隔 10~15 秒推一次
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}}
    )
```

---

## 8. EnvironmentCfg 汇总配置

把以上所有配置组合到一起。

```python
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils

@configclass
class MosEnvCfg(ManagerBasedRLEnvCfg):

    # 场景
    scene: MosSceneCfg = MosSceneCfg(
        num_envs=4096,       # 并行环境数量
        env_spacing=2.5      # 环境间距（米）
    )

    # 各模块
    observations: MosObservationCfg = MosObservationCfg()
    actions: MosActionCfg = MosActionCfg()
    rewards: MosRewardCfg = MosRewardCfg()
    terminations: MosTerminationCfg = MosTerminationCfg()
    events: MosEventCfg = MosEventCfg()

    # 仿真参数
    def __post_init__(self):
        self.sim.dt = 0.005          # 物理步长 200Hz
        self.decimation = 4          # 每 4 个物理步执行一次策略，即策略频率 50Hz
        self.episode_length_s = 20.0 # 每个 episode 最长 20 秒

        self.sim.physics_material = sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        )
```

---

## 9. 注册环境

在扩展包的 `__init__.py` 中注册到 gym：

```python
import gymnasium as gym

gym.register(
    id="Mos-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": "your_package.envs.mos_env:MosEnvCfg",
        "rsl_rl_cfg_entry_point": "your_package.agents.rsl_rl_cfg:MosPPORunnerCfg",
    }
)
```

注册后可以通过任务名启动训练：

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Mos-v0 --num_envs 4096
```

---

## 10. 文件结构

建议的目录组织方式：

```
Mos/
├── assets/
│   └── mos/
│       └── mos.usd
├── articulation/
│   └── mos_cfg.py          # ArticulationCfg（已完成）
└── envs/
    ├── __init__.py          # 注册环境
    ├── mos_env_cfg.py       # 本文所有 Cfg 放这里
    └── agents/
        └── rsl_rl_cfg.py    # 训练 agent 配置（PPO 参数等）
```

---

## 工作量参考

| 模块 | 难度 | 说明 |
|------|------|------|
| SceneCfg | ⭐ | 套模板即可 |
| ObservationCfg | ⭐⭐ | 选择合适的观测项 |
| ActionCfg | ⭐ | 通常用 JointPositionAction |
| RewardCfg | ⭐⭐⭐⭐ | 最需要调试，影响训练效果最大 |
| TerminationCfg | ⭐⭐ | 需要根据机器人形态设置 |
| EventCfg | ⭐⭐ | 随机化范围需要根据机器人调整 |
| 注册 | ⭐ | 套模板即可 |
