# `run_mos_0509.py` 详解

在 Isaac Sim 中加载 `Mos/assets/mos_0509.usd`，让单条机器狗腿按设定的步态正弦摆动，并实时诊断闭链 (closed-loop) 关节的几何误差。

脚本路径：`Mos/articulation/run_mos_0509.py`
对应资产：`Mos/assets/mos_0509.usd`（其 payload 指向 `Mos/assets/mos_leg_51/configuration/mos_leg_51_physics.usd`）

---

## 1. 运行方式

```bash
./isaaclab.sh -p Mos/articulation/run_mos_0509.py
```

可选参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `--usd` | `Mos/assets/mos_0509.usd` | 要加载的 USD 路径 |
| 其他 | — | `AppLauncher.add_app_launcher_args` 注入的标准参数（`--device`、`--headless` 等） |

启动后会在世界 z=0.6 m 处生成一条腿，每 1000 步（约 16.7 s）reset 一次，无限循环。终端日志包含：

- `[INFO]` 一次性打印：被驱动关节、所有关节名、所有 body 名、loop 端 body 索引
- `[LOOP]` 每 60 步打印一次 shank 端、shank_link 端 anchor 在世界坐标下的位置和距离
- `[INFO] Reset.` 每次重置打印

---

## 2. 机构拓扑（重要）

### 2.1 Body / 关节列表

URDF→USD 转换后的 articulation 有 8 个 body 和 9 个关节（含闭链 + 根关节）：

```
hip_motor ── root_joint ── world (固定)
   │
   └── hip_joint ── hip_body
                       ├── font_gear (joint) ── front_gear (body)
                       ├── rear_gear (joint) ── rear_gear (body)
                       ├── shank_gear (joint) ── shank_gear (body) ── shank_link (joint) ── shank_link (body)
                       └── thigh_gear (joint) ── thigh (body) ── shank_middle (joint) ── shank (body)
                                                                                              │
                                                                       shank_loop (joint, 闭链)│
                                                                                              ↓
                                                                                         shank_link
```

四个齿轮 (`front_gear`, `rear_gear`, `shank_gear`, `thigh`) 都是 `hip_body` 上的同心铰，各自独立旋转。`shank` 通过 `shank_middle` 挂在 `thigh` 下，`shank_link` 通过 `shank_link` 关节挂在 `shank_gear` 下，二者末端通过 `shank_loop` 闭合形成 4-bar 平面机构。

### 2.2 关节角色分类

| 类别 | 关节 | 由谁控制 |
|---|---|---|
| **驱动 (Driven)** | `hip_joint`, `font_gear`, `rear_gear` | 脚本里 `set_joint_position_target` |
| **Mimic（齿轮啮合，被动跟随）** | `shank_gear`, `thigh_gear` | USD 里 `PhysxMimicJointAPI`，约束 `q_target = -q_ref` |
| **被动 (Passive)** | `shank_middle`, `shank_link` | 由闭链约束 + 重力自然驱动 |
| **闭链 (Loop)** | `shank_loop` | maximal-coordinate 约束（`excludeFromArticulation=True`），把 `shank` 末端和 `shank_link` 末端拉在一起 |

---

## 3. 齿轮啮合实现：PhysxMimicJointAPI

物理上：
- `font_gear` 与 `shank_gear` 外啮合（1:1）
- `rear_gear` 与 `thigh_gear` 外啮合（1:1）

在 USD 里**没有**用 `PhysxPhysicsGearJoint`（那种需要 `excludeFromArticulation=True`，会破坏 articulation 树），而是用 PhysX 推荐的 `PhysxMimicJointAPI`（一个 multi-apply API schema）。

### 3.1 约束公式

PhysxMimicJointAPI 强制：

```
q_target  +  gearing × q_reference  +  offset  =  0
```

我们用：`gearing = 1.0`, `offset = 0.0`，所以：

- `q_shank_gear = -q_font_gear`
- `q_thigh_gear = -q_rear_gear`

### 3.2 在 USD 里的写法

由 `add_loop_joint.py` 之外的一段独立代码（直接 pxr Sdf 操作）写入。生效后 `/mos_leg_51/joints/shank_gear` 会带上：

```
apiSchemas: [PhysxMimicJointAPI:rotY, ...]
physxMimicJoint:rotY:referenceJoint -> /mos_leg_51/joints/font_gear
physxMimicJoint:rotY:referenceJointAxis = "rotY"
physxMimicJoint:rotY:gearing = 1.0
physxMimicJoint:rotY:offset = 0.0
```

`thigh_gear` 同理，引用 `rear_gear`。

### 3.3 为什么驱动 `font/rear` 而不是 `shank/thigh`

按 mimic 公式，`shank_gear` 和 `thigh_gear` 已经是被约束的从动 DOF，articulation 不会把它们当作可驱动的独立 DOF。所以脚本里：

- **驱动**：`hip_joint` + `font_gear`（带动 `shank_gear` 反向）+ `rear_gear`（带动 `thigh_gear` 反向）
- `shank_gear` 和 `thigh_gear` 放在 `PASSIVE_JOINTS`，stiffness/damping 设 0，确保 URDF→USD 默认的 drive 不会跟 mimic 打架

---

## 4. 闭链 `shank_loop` 标定值

### 4.1 USD 里的存储

```
joint:    /mos_leg_51/joints/shank_loop
type:     PhysicsRevoluteJoint
axis:     Y
body0:    /mos_leg_51/shank
body1:    /mos_leg_51/shank_link
localPos0 (in shank frame):       (-0.06813,   0.00000,   0.10380)
localPos1 (in shank_link frame):  (-0.08510,   0.00385,  -0.13561)
excludeFromArticulation: True   ← 闭链关节强制外置
```

### 4.2 这两个值是怎么来的

| 步骤 | 来源 | 值 |
|---|---|---|
| 初始估计 | STL bbox 角点猜测（`add_loop_joint.py` 里的 `estimate_tip`） | 误差 ~10 mm |
| 在 GUI 中手动校准 | Isaac Sim Stage 面板里直接拖参数到目视对齐 | (-0.06813, 0, 0.1038) / (-0.0851, 0, -0.13561) |
| Y 方向再补 | 诊断打印发现稳态 ΔY ≈ -3.85 mm，由于机构绕 Y 旋转不改变 Y 分量，这个偏差是 URDF 链 Y 偏移的固定误差，把 `pos1.y` 加 +0.00385 一次性消除 | (-0.0851, **0.00385**, -0.13561) |

### 4.3 `LOOP_POS0` / `LOOP_POS1` 在脚本里的作用

脚本头部的 `LOOP_POS0`、`LOOP_POS1` 常量**只用于诊断打印**，不修改物理。物理引擎实际生效的是 USD 里的 `localPos0/1`。如果要真正改约束，要直接改 USD（或重新跑 `add_loop_joint.py`）。

### 4.4 残差水平

- **改 Y 之前**：稳态 |Δ| ≈ 4.5 mm（Y 占 3.85 mm）
- **改 Y 之后**：稳态 |Δ| ≈ 1.3–1.8 mm，剩下的是 X-Z 残差（部分恒定 + 部分跟姿态相关，源于 URDF 链长不完全闭合，无法用 anchor 修正彻底消除）

---

## 5. 步态参数

每个驱动关节按 `q(t) = amp · sin(2π · freq · t + phase)` 摆动。

```python
name_to_params = {
    "hip_joint":  (0.25,  0.20, 0.0),       # ±0.25 rad，0.2 Hz
    "rear_gear":  (-0.40, 0.15, math.pi/4), # 经 mimic 让 thigh_gear  =  +0.40·sin(...)
    "font_gear":  ( 0.40,-0.15, math.pi/2), # 经 mimic 让 shank_gear  =  -0.40·sin(...)
}
```

mimic 反相规则：要想 `shank_gear` / `thigh_gear` 跟原来步态一致，需要 `font_gear` / `rear_gear` 的目标值取**相反符号**（这就是为什么 `font_gear` 是 `+0.4`，`rear_gear` 是 `-0.4`）。

### 5.1 关节 limit 边界

| 关节 | URDF 限位 | 当前峰值 | 余量 |
|---|---|---|---|
| `hip_joint` | ±0.5 rad | ±0.25 | 50% |
| `font_gear` / `rear_gear` | ±0.5 rad | ±0.4 | 20% |
| `shank_gear` / `thigh_gear`（受 mimic 反相） | ±1.0 rad | ±0.4 | 60% |

加大 `font_gear` / `rear_gear` 振幅时要顾及自身 ±0.5 rad 限位，同时观察是否引起 shank 部分穿模。

---

## 6. 物理仿真参数与原因

| 参数 | 值 | 为什么 |
|---|---|---|
| `solver_position_iteration_count` | **64** | 闭链 + mimic 都吃迭代数；默认 8 时稳态闭链残差 >5 mm |
| `solver_velocity_iteration_count` | **4** | 提升约束收敛速度，避免高频振荡 |
| `enabled_self_collisions` | False | 同一个机构内部的相邻 body 不需要碰撞检测，省时间 |
| 初始 `pos.z` | **0.6 m** | 0.35 时 shank 末端会擦地，提高生成位置避免初始穿透 |
| 驱动 actuator `stiffness=20, damping=2` | — | 80/4 时驱动太硬，会赢过闭链约束导致不闭合；20/2 比较平衡 |
| 被动 actuator `stiffness=0, damping=0` | — | URDF→USD 默认会给所有关节加 `stiffness≈1.745` 的位置 drive，会把被动关节锁死，必须显式置零覆盖 |

---

## 7. 诊断输出读法

每秒（60 步）打印一次：

```
[LOOP] t=  3.00s  shank_anchor_w=[ x  y  z]  link_anchor_w=[ x  y  z]  |delta|=1.41 mm
```

- `shank_anchor_w` = `shank` body 世界位姿 把 `LOOP_POS0` 转过来后的世界坐标
- `link_anchor_w` = 同理，但用 `shank_link` + `LOOP_POS1`
- `|delta|` = 二者欧氏距离

判断标准：

| 数量级 | 含义 |
|---|---|
| < 1 mm | 几何完美闭合，求解器够 |
| 1–3 mm | 轻微 URDF 链长不一致，无视觉影响 |
| 3–10 mm | URDF 几何明显有问题，建议复查 SolidWorks 量值 |
| > 10 mm | 求解器 / actuator 调参失衡，或 anchor 标错 |

---

## 8. 常见问题与排查

### 8.1 `shank_middle` 或其他被动关节不动

**原因**：URDF→USD 默认给所有关节加了 `PhysicsDriveAPI:angular`（stiffness≈1.745，target=0），把被动关节锁死。

**修复**：把它们加进 `PASSIVE_JOINTS`，并在 `actuators` 里配一组 `stiffness=0, damping=0` 的 actuator，让 IsaacLab 在运行时覆盖 USD 的 drive 参数。

### 8.2 闭链有可见位移（shank 和 shank_link 拉不到一起）

排查顺序：

1. 看 `[LOOP] |delta|`。如果 >10 mm，先检查 `LOOP_POS0/1` 跟 USD 里 `localPos0/1` 是否一致（脚本里只是诊断常量，不影响物理）。
2. 拉高 `solver_position_iteration_count`（已是 64，可以试 128）。
3. 降低驱动 `stiffness`，减小驱动对闭链的"撕扯"。
4. 用本文 §4.4 的 Y 修正法吃掉恒定方向的残差。
5. 如果 X-Z 也有恒定残差，且方向独立于姿态，说明 URDF 链长有偏差，回 SolidWorks 重新量。

### 8.3 齿轮反向了 / 步态不对

mimic `gearing` 默认 `+1`（外啮合反向）。如果实物是内啮合或带惰轮，应该改为 `-1`。改完要同步改脚本 `name_to_params` 里 `font_gear` / `rear_gear` 的振幅符号。

### 8.4 启动报错 `mimic joint requires limit`

Mimic 要求被 mimic 的 RevoluteJoint 设置了 `lowerLimit` / `upperLimit`。`shank_gear` 和 `thigh_gear` 在 USD 里已经是 ±57.3°（≈±1 rad），如果重新导出 URDF 后限位丢失，需要在 URDF 里写 `<limit lower=... upper=.../>` 并用 `revolute` 类型（不是 `continuous`）。

### 8.5 穿模

调小 `name_to_params` 里 shank 端目标值（即 `font_gear` 振幅），或者拉高初始 `pos.z`。

---

## 9. 关联工具

| 文件 | 作用 |
|---|---|
| `add_loop_joint.py` | 把 `shank_loop` 闭链关节写到 `mos_leg_51.usd`（注意：写的是 `mos_leg_51.usd`，不是 `mos_0509.usd`，两者是分开的） |
| `calibrate_loop.py` / `calibrate_loop_from_sw.py` | 基于 URDF FK + SolidWorks 测量值反算 anchor，给 `add_loop_joint` 提供 `--pos0/--pos1` |
| `loop_joint_calibration.md` | 闭链关节标定方法的总文档 |

---

## 10. 关键事实速查

- 驱动入口：`hip_joint`, `font_gear`, `rear_gear`
- 齿轮 mimic：`shank_gear ← font_gear` / `thigh_gear ← rear_gear`，外啮合 1:1，gearing=+1
- 闭链：`shank_loop`（PhysicsRevoluteJoint, axis=Y, excludeFromArticulation=True）
- 闭链 anchor：`pos0=(-0.06813, 0, 0.1038)`、`pos1=(-0.0851, 0.00385, -0.13561)`
- 物理：64 + 4 求解迭代，actuator 20/2，初始 z=0.6 m
- 诊断：每秒打印 shank/shank_link anchor 世界坐标差，目标 < 2 mm
