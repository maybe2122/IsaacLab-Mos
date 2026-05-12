# 从 URDF 到"机器人能站起来"

把一份新导入的 URDF 跑到能在 Isaac Sim 里站住，需要做的工作分成 6 步，外加一段排错清单。本文档以 `Mos/urdf/mos2026/` 这次的实践为蓝本，所有命令都直接可复制。

> 适用范围：URDF 是从 SolidWorks / Onshape / Fusion 等 CAD 工具导出，结构上不复杂（无 mimic、无显式 loop closure）的足式机器人。带闭链机构的会在第 6 步专门讨论。

---

## 0. 总览：6 步流程

| 步骤 | 输入 | 输出 | 目的 |
|---|---|---|---|
| 1. 摸清 URDF | `*.urdf` | 关节/链接清单 | 知道有多少个 joint、哪些是驱动 |
| 2. 修补 URDF | `*.urdf` | `*_fixed.urdf` | 处理 effort=0、错标 fixed、轴向 (0,0,0) 等 SolidWorks 导出 bug |
| 3. URDF→USD | `*_fixed.urdf` | `assets/<name>.usd` 套件 | IsaacLab 能直接 `UsdFileCfg` 加载 |
| 4. 写 runner | USD | `run_<name>.py` | 设 driven/passive、init pose、spawn 高度 |
| 5. 静态调试 | runner | GUI 看姿态 | 用 `fix_root_link=True` 吊在空中看默认姿态对不对 |
| 6. 落地验证 | runner | 站住 / 走起 | 关掉 fix_base，调 init 姿态 + spawn 高度 |

---

## 1. 摸清 URDF

### 1.1 关节数量与类型

```bash
grep -oE 'type="[a-z]+"' Mos/urdf/mos2026/urdf/mos2026.urdf | sort | uniq -c
#  2 type="fixed"
# 26 type="revolute"
```

- `revolute` 是普通驱动关节。
- `fixed` 会在 URDF→USD 时被 merge 进父链（除非加 `--merge-joints` 时去掉了 merge 行为）。出现在意料之外的位置通常是导出 bug。
- `continuous`、`prismatic`、`floating` 同样要点一遍。

### 1.2 关节名 / 父子链接 / 轴 / 限位

最快的方式是直接用 `awk` 拉出每个 joint 的关键字段：

```bash
grep -nE '<joint|name=|<axis|<limit|type="' Mos/urdf/mos2026/urdf/mos2026.urdf | head -80
```

每一个 joint 关注 5 件事：

| 字段 | 含义 | 出问题的现象 |
|---|---|---|
| `type` | revolute / fixed | 应该转的关节写成 fixed → 不能驱动 |
| `axis xyz` | 旋转轴（在父 link 帧下） | `0 0 0` → PhysX 创建不了驱动 |
| `parent` / `child` | 拓扑 | 父子顺序错就抓不到正确 link 做 sensor |
| `limit effort` | 最大力矩 (N·m) | **0 等价于无驱动**，不是 unlimited |
| `limit velocity` | 最大角速度 (rad/s) | 同上 |
| `origin rpy` | 关节 frame 相对父的旋转 | 全 `0 0 π` 会让"local 左右"和"world 左右"互换，符号会很反直觉 |

> **mos2026 实例**：26 个 revolute 里只有 FL 那 7 个 effort/velocity 是 30，其它 19 个全部是 0；并且 RR 的 `rr_thigh_motor`、`rr_thigh` 被错标为 `fixed`。两个都属于 SolidWorks Exporter 的常见输出 bug。

### 1.3 闭链结构识别

URDF 本身是树形结构，**表达不了闭链回路**。如果你的机构有四杆 / 平行四边形 / 齿轮副，URDF 通常的处理方式是：

1. 在每个回路的某个位置"剪断"，剩下一颗树
2. 剪断处变成两个独立 link，由设计者自己在仿真器里加约束（PhysX joint / mimic）

判断方法：
- CAD 中能看到的回路，URDF 关节数会比"完全打开"的少一个
- 名字里出现 `*_link_a`、`*_link_b`、`*_motor_gear` 通常就是剪断后的悬空链

> mos2026 每条腿名义有 7 个 revolute（hip + 2 motor_gear + thigh + shank + shank_link_a + shank_link_b），但物理上应是 1 DoF（4-bar + 齿轮副，所有动作由 hip + thigh + shank 决定），多出来的 4 个属于剪断后的自由 DoF。后面写 runner 时要把它们设成 passive。

---

## 2. 修补 URDF

如果第 1 步发现以下任一现象，**先在 URDF 阶段修掉**比在 USD 里补容易得多：

| 现象 | 修法 |
|---|---|
| 关节 `effort="0"`、`velocity="0"` | 改成 CAD 里电机标称值（mos2026 用 30 / 30） |
| 关节 `type="fixed"` 但应是 revolute | 改成 `revolute` 并补一个非零 `axis` |
| `axis="0 0 0"` | 按对称腿的轴向补回来 |
| `<inertia>` 全 0 | 给个小数（如 1e-4），否则 PhysX 当无质量处理 |
| mesh 路径用 `package://` 但没有 ROS workspace | 转换前确保 `package://<pkg>/` 能解析到本地，或改成相对路径 |

推荐写一个 **可重跑** 的修补脚本（不要直接改原 URDF），让下次 CAD 重新导出后还能一键打 patch：

```python
# Mos/urdf/mos2026/fix_urdf.py
JOINT_RE = re.compile(r'(<joint\s+name="(?P<name>[^"]+)"\s+type=)"fixed"(?P<body>.*?)</joint>', re.DOTALL)

def patch(text: str) -> str:
    def repl(m):
        if m.group("name") not in FIXED_TO_REVOLUTE:
            return m.group(0)
        ax = FIXED_TO_REVOLUTE[m.group("name")]
        body = AXIS_RE.sub(f'<axis xyz="{ax[0]} {ax[1]} {ax[2]}"/>', m.group("body"))
        return f'{m.group(1)}"revolute"{body}</joint>'

    text = JOINT_RE.sub(repl, text)
    text = text.replace('effort="0"', 'effort="30"')
    text = text.replace('velocity="0"', 'velocity="30"')
    return text
```

> 输出文件命名 `*_fixed.urdf`，原文件保留——这样 CAD 工程师重新导出时不会被吓到。

---

## 3. URDF → USD

IsaacLab 自带的转换脚本：

```bash
./isaaclab.sh -p scripts/tools/convert_urdf.py \
    Mos/urdf/mos2026/urdf/mos2026_fixed.urdf \
    Mos/assets/mos2026.usd \
    --merge-joints
```

关键参数：

| 参数 | 何时用 |
|---|---|
| `--merge-joints` | 把 `type="fixed"` 的关节合并到父链。基本都该开 |
| `--fix-base` | 让 root link 默认固定。**调试用**，跑步态时关闭 |
| `--joint-stiffness 100 --joint-damping 1` | 给所有关节一个默认 PD 增益。后面 runner 里会用 `ImplicitActuatorCfg` 覆盖，所以不用太纠结 |

成功后会得到一个 4 文件套件：

```
Mos/assets/
├── mos2026.usd                    # 顶层引用文件，runner 加载这个
├── config.yaml                    # 转换参数留档
└── configuration/
    ├── mos2026_base.usd           # 视觉网格（大头）
    ├── mos2026_physics.usd        # collider、joint、actuator drive
    ├── mos2026_robot.usd          # articulation root API
    └── mos2026_sensor.usd         # sensor 占位
```

⚠️ **转换后 PhysX 会给每个 joint 加一个默认的 `position drive`**，stiffness ≈ 100（或转换命令里指定的值）。如果某些关节本应自由（passive、闭链端），后面 runner 必须把它们改成 stiffness=0，否则它们会被默认 drive 锁在 0 角。

---

## 4. 写 runner 脚本

模板：`Mos/articulation/run_mos2026.py`。下面按模块逐项说明应该填什么。

### 4.1 列出 driven / passive 关节

```python
LEGS = ["fl", "fr", "rl", "rr"]
DRIVEN_JOINTS = [f"{lr}_{seg}" for lr in LEGS for seg in ("hip", "thigh", "shank")]

PASSIVE_PATTERNS = [
    r".*_shank_motor(_gear)?",
    r".*_thigh_motor(_gear)?",
    r".*_shank_link(_a)?",
    r".*_shank_link_b",
]
```

经验：
- **每个真正的 DoF 设一个 driven 关节**，stiffness 60、damping 3 是不错的起点（小关节）；力矩较大的人形可能要 200/10。
- **闭链剪断处、齿轮副从动端、所有不该自己动的 revolute**，全部走 passive 通道 (`stiffness=0`)，否则 USD 默认 drive 会把它们锁在 0 角。
- **passive 通道给小 damping**（如 0.5），防止它们落地时空甩振荡、把身体拨翻。
- 命名前后不一时（mos2026 里 `fl_shank_motor_gear` 但其他三条是 `fr_shank_motor`），用正则 `.*_shank_motor(_gear)?` 一次匹配。

### 4.2 关节轴翻号 (LEG_AXIS_SIGN)

URDF 里每条腿的 axis 经常因为 CAD mirror 而正负交替：

| 关节 | FL axis | FR axis | RL axis | RR axis |
|---|---|---|---|---|
| hip | (-1,0,0) | (-1,0,0) | (1,0,0) | (1,0,0) |
| thigh | (0,-1,0) | (0,1,0) | (0,-1,0) | (0,1,0) |
| shank | (0,-1,0) | (0,1,0) | (0,-1,0) | (0,1,0) |

要让"同一个 nominal 角度命令 → 同一个 world-frame 动作"，必须给每条腿乘一个 ±1 符号：

```python
LEG_AXIS_SIGN = {
    "fl": {"hip": +1.0, "thigh": +1.0, "shank": +1.0},
    "fr": {"hip": +1.0, "thigh": -1.0, "shank": -1.0},
    "rl": {"hip": -1.0, "thigh": +1.0, "shank": +1.0},
    "rr": {"hip": -1.0, "thigh": -1.0, "shank": -1.0},
}
```

> 实际命令： `joint_target = LEG_AXIS_SIGN[leg][seg] * (offset + amp * sin(...))`

### 4.3 ImplicitActuatorCfg

```python
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
}
```

⚠️ `effort_limit_sim=0` + `stiffness=0` 才是真正"放任自由"。只设 stiffness=0 是不够的，PhysX 还是会按转换器留下的 effort 上限去 clamp。

### 4.4 ArticulationRootPropertiesCfg

```python
articulation_props=sim_utils.ArticulationRootPropertiesCfg(
    enabled_self_collisions=False,
    solver_position_iteration_count=64,
    solver_velocity_iteration_count=4,
    sleep_threshold=0.005,
    stabilization_threshold=0.001,
    fix_root_link=STATIC_BASE,        # 调试模式开关
),
```

- `solver_position_iteration_count` 给到 32-64 对闭链尤其有效（保证 PD 收敛）
- `enabled_self_collisions=False` 默认就够用，除非你需要小腿和身体不能穿模
- `fix_root_link` 是第 5 步的关键开关

---

## 5. 静态调试：先吊起来看

写完 runner 直接跑，最常见的失败是**根本看不出腿往哪儿伸**——body 在第一帧就翻倒了。所以**先把 base 钉在空中**，看 zero-q 下每条腿的自然垂态：

```python
STATIC_BASE = True               # 一行开关
STATIC_BASE_HEIGHT = 0.5         # 吊离地面 0.5 m
```

启动 GUI 模式：

```bash
./isaaclab.sh -p Mos/articulation/run_mos2026.py
```

观察 3 件事：

1. **每条腿的 thigh 指哪？** 应该是基本朝下 / 前下方。如果朝上，说明初始 thigh 关节角度符号反了。
2. **每条腿的 shank 指哪？** 应该顺着 thigh 继续往下。如果 shank 横向甩出去、或往上翻，说明 shank 关节符号反了，或者 zero-q 时 shank 在自身 frame 下不是朝下方向。
3. **左右两侧对称吗？** 不对称就是 `LEG_AXIS_SIGN` 里某一格符号反了。

### 5.1 用计算量化"脚在哪儿"

如果光看不清，把 foot tip 在 base 帧的坐标打出来：

```python
foot_body, _ = robot.find_bodies(["fl_shank"])
foot_w = robot.data.body_pos_w[0, foot_body[0]].cpu().numpy()
base_w = robot.data.root_pos_w[0].cpu().numpy()
print("foot in base:", foot_w - base_w)
```

期望值通常是 `(±something_X, ±something_Y, -足够大的负 Z)`。如果 Z 是正，那是腿翻上去了。

### 5.2 推算 spawn 高度

吊起来稳定后，量一下 `foot_z - base_z` 的绝对值（mos2026 这次是 0.16 m），加 2-5 cm 余量就是 落地版的 `INIT_BASE_HEIGHT`。

---

## 6. 落地验证

把 `STATIC_BASE = False`，再跑一次。常见情况：

| 现象 | 含义 | 修法 |
|---|---|---|
| 身子直接趴下、四条腿摊开 | 腿是向上伸的，根本没在身下 | thigh init 符号反，或闭链 passive 关节默认值不对 |
| 身子前/后倾倒 | 4 条腿不在同一高度，body 重心偏外 | 检查前后两对腿的 `LEG_AXIS_SIGN["thigh"]` 是否一致 |
| 身子侧翻 | 左/右两对腿不对称 | 检查左右两对腿的 `thigh/shank` 符号 |
| 身子悬空抖动 | spawn 太高、脚没着地 | 减小 `INIT_BASE_HEIGHT` |
| 身子陷地 / 脚穿模 | spawn 太低 | 加大 `INIT_BASE_HEIGHT` |
| 全部对的还是站不稳 | 闭链末端在空甩 | passive 通道加 `damping=0.5~2.0`，或给 passive 关节固定 init joint pos |

### 6.1 关于闭链

如果用第 4 步的"简化驱动"方案（每条腿只驱 hip/thigh/shank、闭链全部 passive 自由），机器人能站、能慢走，但：

- 闭链不闭合 → 视觉上 shank_link_b 会和 shank 错位（看上去有"破绽"）
- 高速运动时 passive 那些 0.05-0.17 kg 的 link 会随机甩动，影响落点精度
- 落地冲击没有正确传到 4-bar 上层（CAD 设计时算的力流被改了）

如果要把闭链做对，需要在 USD 上**手工添加**：

1. `PhysxMimicJointAPI`：让 motor_gear 转 1 度时 thigh 反向转 1 度（齿轮副）
2. Loop closure pin：在 `shank` 末端和 `shank_link_b` 末端各打一个 anchor，加距离约束

具体做法参考 `loop_joint_calibration.md` 和 `run_mos_0509.py` 里的实践。每条腿都要做 1 次（mos_dog 就是 4 次），工作量不小。

### 6.2 步态打开

站稳之后再开 gait，**逐项 ramp**：

```python
THIGH_AMP = 0.0  → 0.1 → 0.2 → 0.3
SHANK_AMP = 0.0  → 0.2 → 0.4 → 0.5
GAIT_FREQ = 0.5  → 1.0 → 1.5
```

一次只调一个参数。任何一项让机器人翻倒，就退回上一档。

trot 步态相位对角同相：

```python
LEG_PHASE = {"fl": 0.0, "rr": 0.0, "fr": math.pi, "rl": math.pi}
```

如果 walk 步态（依次抬一条腿）：4 条腿 0/π/π/2 之间的相位间隔 π/2。

---

## 常见排错对照表

| 终端 / GUI 现象 | 大概率原因 | 检查项 |
|---|---|---|
| 启动时 articulation 关节数比 URDF 少 | fixed joint 被 merge | `--merge-joints` 是否打开、原 URDF 里有没有错标 fixed |
| `Number of joints` 对得上但 driven 索引拿不到 | `joint_names_expr` 正则没匹中 | 打印 `robot.data.joint_names` 对照名字 |
| 某条腿初始就静止不动 | 关节 `effort_limit_sim` 是 0 | 检查 URDF 修补是否覆盖了那条腿 |
| GUI 里 root_link 自由飞行不掉 | `fix_root_link` 没关 / USD 里残留 `ArticulationRootAPI` | 检查 `STATIC_BASE`、`config.yaml` 里的 `fix_base` |
| 反复 reset 关节角不变 | `default_joint_pos` 来自 USD 里的 jointAttrs:state:position，被锁住 | 在 ArticulationCfg 的 `init_state.joint_pos` 显式给值 |
| 闭链端 link 飞天 | 它没在 passive 名单里，被默认 drive 拉到 0 | 把它加进 `PASSIVE_PATTERNS` |
| 步态打开就翻 | amp 过大、freq 过高、damping 不够 | 见 6.2 ramp |

---

## 复制即用：最小化 runner 骨架

```python
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
parser.add_argument("--usd", default="Mos/assets/<NAME>.usd")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app = AppLauncher(args_cli).app

import math, torch
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationContext

DRIVEN = ["fl_hip", "fl_thigh", "fl_shank", "fr_hip", ...]   # 12 个
PASSIVE_PATTERNS = [r".*_motor(_gear)?", r".*_link(_a|_b)?"]

STATIC_BASE = True
INIT_BASE_HEIGHT = 0.5 if STATIC_BASE else 0.18

cfg = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=args_cli.usd,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=64,
            fix_root_link=STATIC_BASE,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0, 0, INIT_BASE_HEIGHT)),
    actuators={
        "driven": ImplicitActuatorCfg(
            joint_names_expr=DRIVEN, stiffness=60.0, damping=3.0,
            effort_limit_sim=30.0, velocity_limit_sim=10.0,
        ),
        "passive": ImplicitActuatorCfg(
            joint_names_expr=PASSIVE_PATTERNS, stiffness=0.0, damping=0.5,
            effort_limit_sim=0.0, velocity_limit_sim=1000.0,
        ),
    },
)

sim = SimulationContext(sim_utils.SimulationCfg(device=args_cli.device))
sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
sim_utils.DomeLightCfg(intensity=3000).func("/World/light", sim_utils.DomeLightCfg(intensity=3000))
robot = Articulation(cfg)
sim.reset()

while app.is_running():
    # 不打 gait，先看姿态
    robot.write_data_to_sim()
    sim.step()
    robot.update(sim.get_physics_dt())
```

---

## 参考

- 单腿闭链 + mimic 完整示例：`Mos/articulation/run_mos_0509.py`、`doc/run_mos_0509.md`、`doc/loop_joint_calibration.md`
- 整狗（4 条 mos_0509 拼装）：`Mos/articulation/run_mos_dog_torso.py`、`compose_mos_dog.py`
- 当前 mos2026（简化版四足）：`Mos/articulation/run_mos2026.py`、`Mos/urdf/mos2026/fix_urdf.py`
- IsaacLab URDF importer 官方文档：<https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/ext_isaacsim_asset_importer_urdf.html>
