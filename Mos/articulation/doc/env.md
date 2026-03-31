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