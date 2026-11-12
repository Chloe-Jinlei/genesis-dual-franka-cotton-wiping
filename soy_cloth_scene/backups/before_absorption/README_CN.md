# 双 Franka、酱油与棉布桌面场景

本目录使用本地 Genesis 安装，包含一张桌子、两台独立的 Franka Panda、可变形长方形棉布和原生 SPH 液体酱油。机器人初始保持张开夹爪的待机姿态。

桌面也有 **双 Franka · 酱油与棉布** 启动图标。它会打开交互窗口，运行日志在 `output/viewer.log`，实时运行的预览和状态在 `output/live/`。

## 启动与检查

打开交互窗口（默认 CUDA；关闭窗口结束）：

```bash
/home/chloe/Desktop/genesis-local/run-soy-cloth.sh
```

无窗口运行 500 步，输出状态、图片和检查报告：

```bash
/home/chloe/Desktop/genesis-local/run-soy-cloth.sh --headless --steps 500
```

记录液体运动，并在初始沉降后给液体一个 0.08 m/s 的横向速度：

```bash
/home/chloe/Desktop/genesis-local/run-soy-cloth.sh --headless --steps 750 --record --flow-probe
```

`--flow-probe` 是检验液体响应的外加速度，不代表机器人擦拭。`--record` 必须搭配 `--steps`。默认每步 0.004 秒，500 步对应 2 秒仿真时间；录制按约 30 帧/秒的仿真时间保存，运行耗时取决于设备。首次运行需编译计算内核。

其他选项：`--backend cpu` 使用 CPU；`--liquid-view particle` 查看液体粒子；默认 `recon` 使用原生表面重建。`--config /绝对路径/config.json` 读取另一份配置；`--output /绝对路径/输出目录` 避免覆盖默认输出。

`--check-reset --steps 2500` 可进行 10 秒稳定性检查，并验证场景复位。

默认输出在本目录 `output/`：

- `overview.png`、`detail.png`、`top.png`：初始沉降后的三视角；`final_*.png` 为结束时视角。
- `validation.json`、`final_validation.json`：粒子坐标有限性、桌面穿透、液体位移和机器人待机姿态误差检查。是否通过以实际报告的 `passed` 字段为准。
- `settled_state.npz`：液体和棉布粒子位置、两台机械臂配置。
- `liquid_motion.mp4`：启用 `--record` 后的近景视频。

这些检查用于发现明显的场景问题，不等同于物理模型或操作任务验证。

## 尺寸与配置

位置和尺寸单位为米，Z 轴向上，角度为弧度。可调项在 `config.json`；棉布尺寸和酱油初始网格轮廓由 `generate_assets.py` 生成。

| 项目 | 默认设置 |
| --- | --- |
| 桌面 | 1.60 × 1.00 m，厚 0.06 m，上表面 Z=0.76 m |
| 机械臂基座 | 左 `(-0.57, 0.22, 0.78)`；右 `(0.57, 0.22, 0.78)` |
| 机械臂自由度 | 每台 7 个臂关节 + 2 个夹爪指关节 |
| 棉布 | 0.28 × 0.18 m，41 × 27 网格，中心 XY=`(0.23, -0.20)` |
| 酱油初始轮廓 | 约 0.20 × 0.14 m，中心 XY=`(-0.13, -0.20)` |
| 酱油初始厚度 | 原网格最大 0.002 m × `soy.depth_scale`；默认 3.0，即最大 0.006 m |
| 液体粒子间距 | 0.003 m；采用 `SPH.Liquid`、`WCSPH` 和规则体积采样 |
| 仿真步长 | 0.004 秒，每步 40 个子步 |

棉布以 `PBD.Cloth` 模拟，带棉织纹理，面密度默认 0.22 kg/m²。酱油是随求解器推进的液体粒子，其显示表面由粒子重建；初始网格仅用于生成液体分布。

Genesis 会对输入棉布网格重网格化；当前配置实际得到 1543 个布料粒子和 4438 个液体粒子。

当前密度、黏度、表面张力、摩擦及布料顺应性参数均为示例值，尚未依据特定酱油或棉布实验标定。PBD 的 0.006 m 粒子尺寸对应约 3 mm 接触半径，是较粗的碰撞近似，不是实测布厚。

## 后续控制与当前边界

`scene.py` 的 `build_scene(config, ...)` 返回 `(scene, handles)`。其中 `handles["robots"]["left"/"right"]` 为两台机器人，`handles["robot_dofs"]` 为对应关节索引，`handles["cloth"]`、`handles["soy"]` 为布料和液体，另含桌面、相机和初始状态句柄。`reset_scene(scene, handles)` 可恢复初始状态及双臂控制目标。下面仅读取数据，不执行抓取或擦拭：

```python
import json
from pathlib import Path
import genesis as gs
from scene import build_scene

config = json.loads(Path("config.json").read_text())
scene, handles = build_scene(config, headless=True)
try:
    left = handles["robots"]["left"]
    q = left.get_dofs_position(handles["robot_dofs"]["left"])
    cloth_xyz = handles["cloth"].get_particles_pos()
    soy_xyz = handles["soy"].get_particles_pos()
    print(q.detach().cpu().numpy())
    print("布料/液体位置数组：", cloth_xyz.shape, soy_xyz.shape)
finally:
    gs.destroy()
```

在本目录内、使用本地虚拟环境运行该示例。机器人公开接口包括 `control_dofs_position()` 和 `inverse_kinematics()`；粒子实体可读取 `get_particles_pos()` 并设置 `set_particles_vel()`。PBD 还提供粒子固定接口；这些是实验接口，本场景尚未提供抓布、轨迹规划或擦拭控制器。

场景启用了刚体与 SPH、刚体与 PBD 的接触；**没有 SPH↔PBD 耦合、吸水、染色或酱油移除模型**，因此布经过酱油时不应解释为已实现真实擦拭。当前安装的原生 PBD↔刚体接触实现也未提供摩擦，配置中的布料摩擦值不能保证其在桌面或夹爪上不滑动。官方[耦合说明](https://genesis-world.readthedocs.io/en/v1.4.0/user_guide/theory/coupling/index.html)列出了默认求解器支持的耦合组合，其中不包含 SPH↔PBD。

## 本机验证记录

2026-11-10，在 Genesis 1.4.3 / CUDA 上完成：

- 2500 步、10 秒仿真：粒子保持有限，液体和布料留在桌面上，双臂保持待机姿态，场景复位通过。报告为 `output/final_validation.json`。
- 独立 750 步流动检查：施加 0.08 m/s 横向初速度后，液体质心沿 X 移动约 3.57 mm，检查通过。报告与视频为 `output/flow_check/final_validation.json` 和 `output/flow_check/liquid_motion.mp4`。
- 初始布局预览为 `output/overview.png`，近景为 `output/detail.png`。

上述结论仅针对当前配置和静置/轻推实验，不表示已验证抓取或擦拭任务。
