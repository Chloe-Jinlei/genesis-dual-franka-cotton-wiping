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

`--flow-probe` 是检验液体响应的外加初速度，不代表机器人擦拭。常规模式的 `--record` 必须搭配 `--steps`。默认每步 0.004 秒，500 步对应 2 秒仿真时间；录制按约 30 帧/秒的仿真时间保存，运行耗时取决于设备。首次运行需编译计算内核。

验证棉布扫过酱油、吸液并抬起携带，保存 12 秒的仿真视频：

```bash
/home/chloe/Desktop/genesis-local/run-soy-cloth.sh --headless --demo-wipe --record
```

演示由规定运动的两条布边夹具驱动，其余布节点仍可变形，Franka 暂时保持待机；这是吸液耦合验证，尚非机械臂自主抓布擦拭。去掉 `--headless` 可观看交互窗口。输出默认在 `output/wipe_demo/`：

桌面的 **棉布吸液演示（受控夹具）** 图标也可直接启动此交互演示，独立输出在 `output/wipe_live/`，不会覆盖已验证的视频。

- `wipe_fixture_demo.mp4`：扫过、吸液、抬起的仿真记录。
- `before_*.png`、`after_sweep_*.png`、`lifted_*.png`：三个阶段的画面。
- `wipe_validation.json`：质量平衡、湿布质量、局部容量、携带与复位检查。
- `mass_history.csv`：桌面自由液体和布内液体的质量随时间变化。
- `final_coupled_state.npz`：结束时的诊断快照，不能直接用来恢复仿真；续跑应使用下述完整耦合状态接口。

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

## 吸液与携带模型

当前默认开启 `coupling.enabled`。原生 Genesis 未提供这一对求解器的直接耦合，因此本场景通过本地扩展 `cloth_liquid_coupling.py`，在每个 4 ms 物理步结束后、显示更新前处理布与液体的接触、拖曳和吸收，不改动 Genesis 安装源码。`scene.step()` 会自动执行这一层。

接触检查使用变形后布料物理三角形上的最近点及重心坐标。只有接近实际布面、且持续接触达到阈值的液体才有机会吸收；每个布节点都有按面积计算的含液容量和吸收通量。无接触时不会积攒未来的吸收额度，饱和后不会继续无限吸收。

吸收后，该 SPH 液体粒子退出自由流动求解，但其质量保存在布内液体记录中，按三角形重心权重加入真实 PBD 节点质量；因此湿布的惯性、重力负载和约束中的逆质量随之变化。记录中的液体位置始终跟随布面，抬起后不会被记为凭空消失。局部饱和度会将原棉织纹理染深，染色随布变形。

接触拖曳和质量转移在各自算子中保持自由布节点与液体的线动量平衡；固定夹具承担的反作用单独计入报告。几何防穿透采用单独的位置投影。默认参数：

| 参数 | 默认值 |
| --- | --- |
| 面积含液容量 | 1.2 kg/m²，整块布约 60.48 g 液体 |
| 干布质量 | 约 11.09 g |
| 初始自由酱油质量 | 按原生粒子质量计算，约 105.45 g |
| 未饱和时最大吸收通量 | 1.0 kg/(m²·s)，随饱和度增加而降低 |
| 最短连续接触时间 | 0.035 s |
| 吸液接触范围 | 3.5 mm，适应当前粒子分辨率 |
| 法向接触距离 | 1.5 mm |
| 切向拖曳速率 | 25 s⁻¹ |

上述是可调、未标定的介观多孔布近似；它没有解析棉纤维孔隙、毛细压力、布内扩散、蒸发、挤水、解吸或酱油化学染色。默认容量小于酱油总量，单次擦拭不应预期完全清除所有液体。当前只支持单环境，耦合层不可微分。原生 PBD 与桌面/夹爪的切向摩擦仍需独立处理，不能将本次布—液体拖曳等同于可靠抓布。

程序接口：`handles["coupling"].metrics()` 返回自由液体、吸收量、容量、质量和动量残差；`vertex_saturation`、`vertex_absorbed_mass`、`bound_positions` 提供逐节点湿度、含液量及被携带液体位置。`scene.reset()` 或 `reset_scene(scene, handles)` 会恢复干布质量、液体 active 状态、接触计时和初始位置。保存与恢复湿场景需同时保留两个求解器与吸液记录：

```python
from scene import capture_coupled_state
checkpoint = capture_coupled_state(scene, handles)
# ... continue simulation ...
scene.reset(checkpoint)
```

该检查点是当前进程中的完整状态对象，不能用单独的原生 `scene.get_state()` 替代。实现参考的是粒子质量守恒和薄壳接触的一般原则，并非复现某篇完整算法；可参阅官方[求解器耦合说明](https://genesis-world.readthedocs.io/en/v1.4.0/user_guide/theory/coupling/index.html)和[薄壳与流体耦合研究](https://www.graphics.stanford.edu/papers/thin_shells_fluid_coupling-sig05/)。

## 本机验证记录

2026-11-10，在 Genesis 1.4.3 / CUDA 上完成：

- 2500 步、10 秒仿真：粒子保持有限，液体和布料留在桌面上，双臂保持待机姿态，场景复位通过。报告为 `output/final_validation.json`。
- 独立 750 步流动检查：施加 0.08 m/s 横向初速度后，液体质心沿 X 移动约 3.57 mm，检查通过。报告与视频为 `output/flow_check/final_validation.json` 和 `output/flow_check/liquid_motion.mp4`。
- 初始布局预览为 `output/overview.png`，近景为 `output/detail.png`。

以上三项是添加吸液前的静置/轻推基线。吸液扩展的独立模型回归测试位于 `tests/test_coupling_model.py`，擦拭运行结果以 `output/wipe_demo/wipe_validation.json` 为准；两者都不等同于机器人抓取策略验证。

添加吸液后的 12 秒受控扫过/抬起测试已通过：初始酱油 105.45 g，其中 34.00 g（32.24%）转入布内，剩余自由液体 71.45 g。湿布的原生模拟质量由 11.09 g 增至 45.09 g。吸收液体随布抬高的中位数为 11.12 cm，液体总质量账目误差为 0；原生湿布质量与吸收记录的最大差约为 3.64e-11 kg。局部饱和度未超过 1，完整复位通过。该演示约束 55 个布边节点，其余 1488 个节点保持柔性动力学。

12 项模型回归测试可用下列命令重跑：

```bash
/home/chloe/Desktop/genesis-local/.venv/bin/python -m unittest discover -s /home/chloe/Desktop/genesis-local/soy_cloth_scene/tests -v
```

另已通过原生 GPU 边界检查：测试专用高容量/高吸速参数下，4438 个液体粒子全部转入布内后仍可正常渲染；湿场景检查点可恢复位置、湿质量及 inactive 状态；复位可恢复干布、全部自由液体与干燥外观。报告为 `output/coupling_edge_checks/validation.json`，测试参数只用于该边界检查，未写入默认 `config.json`。
