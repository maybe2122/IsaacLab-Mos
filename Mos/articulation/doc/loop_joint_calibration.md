# 闭链铰孔标定 —— 我需要你帮我测的参数

我现在没法直接看 GUI，所以闭链 anchor 的真实位置得你帮我量。下面任选一种方式都行（**A 最省事，只测 1 个数**）。

---

## 方式 A：在标定姿态下读 1 个世界坐标（推荐）

### 前置条件

把 7 个关节固定到你之前截图里那个 "正确合拢" 配置：

| Joint | 角度 (rad) |
|---|---|
| `hip_joint` | 0.14 |
| `font_gear` | 0.14 |
| `rear_gear` | 0.08 |
| `thigh_gear` | 0.27 |
| `shank_middle` | -0.07 |
| `shank_gear` | -0.22 |
| `shank_link` | -0.05 |

> 用 GUI 里的 Joint Editor 把这 7 个值都拨到上面的数（保持物理暂停或没启动），让 shank 末端孔和 shank_link 末端孔在视觉上对齐。

### 我要的数据

闭链铰孔在**世界系**下的 1 个 3D 坐标 `(x, y, z)`，单位米。

### 怎么测

任选一种：

1. **Measure 工具**（最快）：菜单 Window → Utilities → Measure。激活后用 Single Point 模式，点闭链孔中心，面板里会显示 World Position。
2. **临时 marker**：Stage 里 Create → Xform，在 viewport 把它拖到孔中心，然后选中它，Property 面板里 `xformOp:translate` 就是世界坐标（前提是这个 Xform 直接挂在 `/World` 下，而不是任何机器人 link 的子级）。
3. **选 mesh 顶点 / 面**：选中 `/World/Origin1/Robot/shank` 或 `shank_link` 下的 mesh，进 Mesh 子选择模式选孔附近的一个顶点，Property 面板能看到 World position。

### 报给我的格式

只要这一行就够：

```
world_xyz = (X, Y, Z)
```

例：`world_xyz = (-0.21, -0.05, -0.31)`

我会自动用 FK 反推出 shank 与 shank_link 各自本地坐标系下的锚点，写回 USD。

---

## 方式 B：直接给两个本地坐标（不用 FK）

如果你能在 GUI 里分别读出 shank 末端孔在 shank link 本地系、shank_link 末端孔在 shank_link link 本地系下的 (x,y,z)，我就直接写进 USD 不再算 FK。

### 怎么测（每个 link 一遍）

1. Stage 里选中 `/World/Origin1/Robot/shank`（或 `shank_link`），右键 Create → Xform，新 Xform 会成为该 link 的子级。
2. 在 viewport 里把 Xform 拖到对应的孔中心。
3. 选中这个 Xform，Property 面板里 `xformOp:translate` 就是它在父 link 局部坐标系里的 (x,y,z) ——**正是我需要的本地锚点**。

### 报给我的格式

```
shank 本地锚点      = (X0, Y0, Z0)
shank_link 本地锚点 = (X1, Y1, Z1)
```

---

## 方式 C：从 CAD 设计图读尺寸

如果你手头有 SolidWorks 装配体或零件图，给我以下两组数（与 URDF mesh 是同一个本地坐标系，单位米）：

- `shank` 末端铰孔中心相对 `shank` link 原点的 (x, y, z)
- `shank_link` 末端铰孔中心相对 `shank_link` link 原点的 (x, y, z)

格式同方式 B。

---

## 顺便确认两件事

1. **铰链轴方向**：当前我设的是绕 Y 轴。如果你的 SolidWorks 里这个铰孔的轴不是 Y，请告诉我（X / Y / Z）。
2. **是否需要限位**：现在闭链是自由旋转。如果该铰孔实际有机械限位（角度范围），告诉我下/上限即可。

---

## 你测好之后我会做的事

1. 用你给的数据，重写 `Mos/assets/mos_leg_51/mos_leg_51.usd` 中 `/mos_leg_51/joints/shank_loop` 的 `localPos0` / `localPos1`（必要时也改 axis 和 limit）。
2. 验证：在标定姿态下两 anchor 在世界系应该几乎重合（误差 < 1 mm）。
3. 你重启 demo 应能看到机构在所有姿态下都正常合拢，不再有跳动/抖动。
