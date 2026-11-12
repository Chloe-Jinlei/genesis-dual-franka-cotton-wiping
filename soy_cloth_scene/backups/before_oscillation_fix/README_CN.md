# 双 Franka 抓布、吸液与提起任务

本目录使用本地 Genesis，包含桌子、两台 Franka Panda、长方形柔性棉布和可流动的 SPH 酱油。`robot_wipe.py` 使用两台机械臂抓住布料的两个对角，执行搬运、铺布、松开、上下按压、同步平移擦拭、重新抓取和提起湿布；吸液量由局部接触、吸收通量和有限容量决定。当前采用用户 PDF 所列的二维机织棉布近似，材料参数尚未实验标定。

当前默认材料已改为用户 PDF 的 `plush_cotton` 棉布模型：180 g/m²、经/纬/斜向 2%/2%/10% 伸长投影目标、B=5×10⁻⁵ N·m 二面角弯曲，使用保留经纬的规则物理网格。来源是只读的 [用户 PDF](/home/chloe/Desktop/熊与棉布_物理参数设置_plush_cotton.pdf) 第 2、4、6 页及对应本地实现。**新版本已完成全部吸液，并从同材料检查点续跑完成湿抓和提起；但干、湿布的经纬伸长仍超标，最终 17/19 项通过，整体 `passed=false`。** 默认已采用该次完成动作的湿抓反馈控制，但没有以新默认从干态再完整跑一轮。本文后面的 `small_pinch` 18 项成功记录属于先前 400 g/m² 模型，不能用作本次新材料验证。

运行 [run-soy-cloth.sh](/home/chloe/Desktop/genesis-local/run-soy-cloth.sh) 可启动双臂任务。该脚本还支持静态场景、液体流动检查和原有受控夹具测试，具体模式见下文。

## 运行双臂任务

打开交互窗口，任务结束后保留窗口：

```bash
/home/chloe/Desktop/genesis-local/run-soy-cloth.sh --robot-wipe --hold-open
```

无窗口完成任务并录制视频：

```bash
/home/chloe/Desktop/genesis-local/run-soy-cloth.sh --robot-wipe --record --headless
```

`--robot-wipe` 自动读取 `config_robot_wipe.json`。这个模式的 `--record` 不需要提供 `--steps`；提供 `--steps` 会限制总步数，提前停止会记录为未完成。首次运行需要编译计算内核，实际运行耗时通常长于视频中的仿真时间。

任务按以下顺序执行：

1. 双臂接近布料的两个对角，在空中先将每个指关节预闭合到 **8 mm**，再下降，减少夹爪把大段材料搓成团块的机会。
2. 夹爪继续闭合。候选节点限定在初始材料坐标中、距对应角部抓点 **22 mm** 内；每侧指垫必须真实接触至少两个节点，但每侧只选一个不同的代表节点固定，即每只手固定两个布节点。固定点对的距离不得比初始材料距离增加超过 **2%**，否则拒绝抓取。首次夹取目标 TCP 高度为 **0.770 m**；接触不足时可下探 0.2 mm，最低为 0.7698 m，接触门限保持不变。
3. 两台机械臂共同提起棉布，搬运到酱油上方并下降铺布。
4. 解除抓取约束并张开夹爪，让布的内部区域在桌面上接触液体。
5. 双臂反复进行低幅度上下按压。每次下压后若仍有自由液体，两手同步沿桌面 **−Y 方向移动 1 cm，再返回**，每段 1.6 s；目标对角间距不变。此时抓取约束已解除，由真实指垫接触和摩擦带动自由布料，尝试让尚有容量的材料接触残液。布内液体同时按局部饱和度差进行有限输运。每个周期读取剩余自由液体粒子数，并记录布料实际位移。
6. 当自由液体粒子数为 **0**、且至少执行两个按压周期后，在 TCP 高度 **0.775 m** 将湿抓开口预闭合至每指 **2 mm**，再下降到 **0.7704 m** 夹取同一对材料角块。每指按 **0.1 mm** 小步闭合，每次用 0.05 s 移动、0.05 s 等待，再检查原有接触和材料门限；各手一旦通过即保持实测开度，另一手可继续闭合。双手均通过后用 **3.5 s** 提起湿布，再稳定 **1 s**。未通过抓取门限时停止；动作完成后仍独立检查整布材料残差。

按压与擦拭循环有配置上限。达到上限仍有自由液体、抓取接触条件不满足、轨迹求解失败，或质量/容量检查失败时，程序会停止并保存失败原因；不会删除剩余液体来满足完成条件。这里的反馈来自仿真粒子状态，尚未实现相机视觉识别或真实传感器测量。

默认输出目录为 `output/robot_wipe/`：

| 输出 | 内容 |
| --- | --- |
| `dual_arm_wipe.mp4` | 双臂全过程；主画面和布面近景、阶段名称、自由液体和布内液体质量 |
| `robot_wipe_validation.json` | 是否完成、剩余粒子数、质量守恒、局部容量、双臂运动、最终抓取与提起检查 |
| `events.json` | 阶段时间、目标位置、实际抓取接触证据、擦拭时布料位移中位数和液量测量 |
| `mass_and_motion.csv` | 液量、饱和度、布料高度及末端跟踪误差随时间变化 |
| `01_initial*.png` 等阶段图片 | 抓布前、抓取后、提起、铺布、按压后和最终湿布画面 |
| `cloth_shape_measurements.json` | 各阶段边伸长的 P95、P99、最大值、经/纬/斜向超限量，以及布面投影和近桌面投影面积；方向依据初始材料坐标 |
| `*_cloth.npz` | 对应阶段的 `positions`、初始 `reference` 与三角形 `faces`，可复核形状和伸长；不是完整恢复检查点 |
| `config_used.json` | 本次实际使用的全部参数 |
| `final_state.npz` | 结束时的诊断数组；不能单独用于恢复仿真 |

`passed: true` 才表示该次报告内的全部检查通过。最终检查还要求湿布整体离开桌面至少 6 cm、被吸收液体随布上升至少 6 cm，以及两台机械臂均有实际关节运动。视频本身不替代这些检查。

可用 `--output /绝对路径/输出目录` 保存独立实验，`--config /绝对路径/config.json` 读取另一套参数。`--liquid-view particle` 显示液体粒子；默认 `recon` 使用原生表面重建。`--backend cpu` 可选择 CPU。

## 双臂任务的尺寸与材料参数

位置单位为米，Z 轴向上，角度为弧度。双臂配置在 `config_robot_wipe.json`；输入布料与液体网格由 `generate_assets.py` 生成。

| 项目 | 双臂任务配置 |
| --- | --- |
| 桌面 | 1.60 × 1.00 m，上表面 Z=0.76 m |
| 机械臂 | 两台独立 Franka Panda，各 7 个臂关节及 2 个指关节 |
| 棉布尺寸 | 0.28 × 0.18 m；显示网格 41 × 27，物理网格 95 × 61（5795 点），保留经纬方向 |
| 棉布网格采样间距 | X 约 2.979 mm、Y 3 mm；禁止随机重新划分，以保留材料方向 |
| 棉布碰撞壳半径 / 总厚度 | `cloth.collision_radius_m=0.00075`，即 0.75 mm / 1.5 mm；尚未实验标定 |
| 棉布初始中心 | XY=`(0, -0.23)` |
| 对角抓点相对布中心 | 左臂 `(-0.13, +0.08)`，右臂 `(+0.13, -0.08)`，各向布内留约 1 cm |
| 初始材料候选半径 | 22 mm；首抓与最终湿布抓取使用同一批材料角部节点 |
| 空中预闭合 | 初抓每指 8 mm；湿抓独立设为每指 2 mm，在 TCP Z=0.775 m 时预闭合 |
| 指关节 PD 增益 | 初抓及湿抓闭合均为 2000/40；湿抓绑定后保持实测开度，改为 10000/100 |
| 首次 / 最终夹取 TCP 高度 | 初抓 0.770 m，湿抓 0.7704 m；接触不足时最多下探一次 0.2 mm，且不低于 0.7698 m |
| 湿抓反馈闭合 | `wet_grasp_feedback_closure=true`；每指 0.1 mm 小步、0.05 s 移动加 0.05 s 等待，原门限通过即保持实测开度 |
| 平移擦拭 | `wipe_translation_xy_m=[0, -0.01]`，两手同步沿 −Y 移动 1 cm 再返回，每段 1.6 s |
| 每只手的接触 / 固定节点 | 每侧指垫至少两个真实接触节点；每侧固定一个不同代表，共两个固定节点 |
| 固定点对伸长上限 | 2%；仅约束建立抓取时选中的固定点对，不代表整块布的所有边始终低于 2% |
| 湿抓绑定前几何 | 候选球壳侵入 pad ≤50 µm；直接固定邻边为 ±2%，候选一环真实网格边为 ±5% |
| 湿布提起 | 3.5 s 提起，随后稳定 1 s |
| 伸展约束 | `stretch_compliance=1e-7 m/N`、松弛 0.3、每子步 10 轮；随后 24 轮经/纬/斜向 2%/2%/10% 伸长投影 |
| 弯曲约束 | `B=5e-5 N·m` 的几何加权二面角模型，松弛 0.5、4 轮；每轮按 17 个互斥四节点组顺序求解，每子步清零乘子 |
| 空气阻力 | 各向同性二次阻力；空气密度 1.225 kg/m³、阻力系数 1.0，各节点按相邻参考三角形面积的三分之一求和 |
| 棉布内部阻尼 | 4 s⁻¹，按实际网格最大节点度数归一化的逐边轴向相对速度耗散 |
| 线性阻力 | 1 s⁻¹，仅作用棉布，避免改变酱油的流变设置；PDF 原场景是全场线性阻力 |
| 棉布静 / 动摩擦 | 0.6 / 0.45；刚体接触取布与刚体系数中较小者 |
| 酱油初始中心 | XY=`(0, 0.04)` |
| 酱油初始轮廓与厚度 | 约 0.10 × 0.084 m；网格最大厚度 0.002 m |
| 酱油缩放 | `soy.xy_scale=[0.5, 0.6]`，`soy.depth_scale=1.0` |
| 液体粒子间距 | 0.002 m，`SPH.Liquid` / `WCSPH` / 规则体积采样 |
| 棉布面密度 | 0.18 kg/m²，整块干布约 9.072 g |
| 局部含液容量 | 1.08 kg/m²，整块布最多约 54.432 g 液体 |
| 干布单位质量含液量 | 6 g 液体 / g 干布，沿用的未标定吸液假设，PDF 没有吸液参数 |
| 最大未饱和吸收通量 | 1.0 kg/(m²·s)，随局部饱和度增加而降低 |
| 最短连续接触时间 | 0.035 s |
| 吸液接触范围 / 法向接触距离 | 3.5 mm / 1.5 mm |
| 切向拖曳速率 | 25 s⁻¹ |
| 布内输运系数 | `D=5e-6 m²/s`，局部饱和度输运的未标定工程值 |
| 仿真步长 | 0.002 s，每步 20 个子步，即每子步 0.0001 s |
| 数值精度 | `simulation.precision="32"`，布料世界坐标 Z 上限设为 1.25 m；构建前检查重力位移分辨率 |

规则采样的小液滩包含 1489 个液体粒子、约 10.48 g 酱油。历史 CPU 验算在平展、持续接触的布面上验证过容量与通量；这不能保证实际折叠、铺布后的局部材料容量足够。实际运行的粒子数、局部饱和度和残液质量以该次报告为准。

容量依据每个节点的参考面积分配，不能从远处未接触的干布区域借用容量。6 g/g、吸收通量 1 kg/(m²·s) 和输运系数 D=5e-6 m²/s 都是独立、未标定的吸液假设，**不是 PDF 提供的材料数据**。新布面积为 0.0504 m²，按 180 g/m² 得到干质量 9.072 g、总容量 54.432 g；原熊场景布料的 29.39 g 来自不同尺寸，不能直接照搬。网格采样间距约 3 mm，碰撞壳总厚度独立设为 1.5 mm；这是接触近似，不是实测布厚。

当前保留原生边伸展求解，再对初始经、纬、斜向网格边施加不同的拉伸投影目标。2%/2%/10% 是有限轮数约束的目标，不是任何时刻都满足的硬限，也不是实测断裂应变；后续弯曲、碰撞和抓取仍会留下超限残差。方向分组提供机织布的简化响应，尚未包含实测非线性应力–应变、纱线卷曲或独立剪切本构。报告按方向列出实际最大伸长与超限量；干/湿提起阶段的方向验收另允许 0.2 个百分点数值残差，不能用旧统一 5% 上限替代。

布料可下垂、起皱和折叠，这些形状变化本身不等于材料被拉长；伸长始终相对初始物理边长计算。斜向 10% 目标作用于网格对角边，为斜向形变保留空间，并非所有方向采用相同的弹性伸长。夹爪附近的过伸长记录仍是需要核查的离散约束/接触残差，不能解释为真实棉布的橡胶式弹性。

空气阻力的每节点系数为 `0.5 × ρ_air × C_d × A_i`，`A_i` 是该节点相邻参考三角形面积之和的三分之一，所有 `A_i` 的和等于布料参考总面积。它不是将面积均分为 `A/N`，也不改变原生均匀干节点质量。此处理避免网格加密放大总阻力；仍是各向同性阻力近似，没有解析布面朝向或透气性。内部阻尼使用互斥边分组，4 s⁻¹ 除以实际最大度数 8 后得到逐边 0.5 s⁻¹；它只耗散轴向相对速度，线性阻力则另外作用于自由棉布节点。各材料算子都读取当前原生质量，吸液后的非均匀增重继续生效。

本次保留三项与参考场景不同的数值设置：参考子步为 1 ms，这里沿用适合现有 SPH 耦合与位置分辨率的 **100 µs**；碰撞直径沿用 **1.5 mm**；参考全场 **1 s⁻¹** 线性阻力限定为只作用棉布，保持酱油动力学参数不变。此外，参考实现的弯曲 Jacobi 同步累加在细网格上会放大交错扰动，现改为四节点互斥分组的 Gauss–Seidel 顺序求解；弯曲能量公式、B=5e-5 N·m、松弛 0.5 和 4 轮均保留。具体对照见下方新材料验证。

`numerical_resolution.py` 在场景构建前要求 `9.81 × h² > 0.5 × ULP`，其中 `h` 为子步时长，ULP 按最大布料世界坐标 Z=1.25 m 计算。当前重力位移约 9.81e-8 m，`ratio=重力位移/ULP≈0.823`，超过最低门限 0.5；运行报告保存该比值，布料超过配置高度范围会停止。该检查只排除重力增量被完全舍入的情形，不证明自由落体精度、接触稳定、流体稳定或全部舍入误差已受控。

## 抓取、按压、擦拭与模型边界

机器人关节通过 Genesis 的 PD 动力学控制，末端路径由逆运动学求解。接触中允许不超过 5 mm 的末端跟踪误差，刚体碰撞另外检查。最低目标 TCP 高度 0.7698 m 在标称姿态下保留约 0.442 mm 指尖与桌面间距，实际接触仍须检查真实链接位置。`robot_grasp.py` 只在指定的初始材料角块中寻找实际接触节点，不借用恰好折叠到附近的其他布料。节点须位于真实夹爪之间且距当前 TCP 不超过 12 mm，两侧指垫各至少接触两个节点；接触距离上限仍为碰撞壳半径加 0.3 mm，即 **1.05 mm**。

初抓中央 pad 间距上限仍为 **4.5 mm**。湿抓显式启用 `preserve_contact_geometry`，中央 pad 间距上限由 **4.5 mm 改为 7.5 mm**：Panda 中央 pad 的内面间距比更靠下的小 pad 间距大 3 mm，因此保留小 pad 间距上限 4.5 mm，对应中央上限 7.5 mm。这个数值门限确实改变，不能称为所有阈值不变；真实双侧支持、材料身份、TCP 距离和拓扑要求继续保留。

两侧接触集合必须通过局部真实网格路径连通。当前连通区域按 25 mm 加碰撞壳半径检查，节点中心距 TCP 上限为 **25.75 mm**，最短布面路径不超过 **51.5 mm**。每侧至少两个真实接触节点的验证通过后，当前配置从各侧选一个不同的代表，形成 **1+1 个固定节点**；两代表之间也必须满足相同的真实网格路径限制。固定点对当前距离不得超过初始材料距离的 1.02 倍，避免锁住已被拉长的材料。没有合格子集时停止并报告，不放宽接触门限。报告分别保留原双侧接触证据、代表节点的连接路径、初始材料包围盒及固定点对伸长。

满足全部条件后，选中节点按当时的局部偏移约束到真实手部链接；建立约束不会把布传送到手中。连通路径中间节点和未选中的真实接触节点保持自由，由原生接触和场景摩擦模型作用，减少角部过约束。2% 检查针对建立约束的固定点对，整块布的局部伸长另由 `cloth_shape_measurements.json` 和阶段 `.npz` 检查。接口的 `minimum_bound_per_finger` 默认仍为 2，当前双臂配置显式传入 1。

湿抓新增筛选要求候选球壳对任一 pad 的侵入不超过 **50 µm**；会触发原生自碰撞的固定点对，其球壳重叠也不得超过该容差。直接连接两个固定点的网格边必须处于参考长度的 **98%–102%**，选中节点一环内所有真实网格边必须处于 **95%–105%**。这避免绑定已经不兼容的局部几何，但不保证之后的接触不会再次卡住。各手绑定后保持当时测得的指关节开度，使用 PD 10000/100，避免继续向零闭合挤压固定节点。当前仍绑定到 `hand`；`finger_pads` 绑定选项仅做过 CPU 测试，生产配置不使用它。

历史 400 g/m² 材料的 [初次夹取接触审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_grip_contact_audit.md) 发现短探针夹取瞬间有 31 条自由节点间的边超过 5%，峰值 **26.7354%**，并存在约 **0.6463 mm** 的局部碰撞壳压入桌面残差。证据支持指尖底部空间不足与顺序接触投影造成局部网格畸变，但尚无修改几何后的对照仿真证明。该问题没有被低伸长抓取门限完全消除；任务及干/湿布提起检查通过，也不能表述为全过程所有边均低于 5%。

同一旧材料的 [湿布夹持审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_wet_grip_conflict_audit.md) 还发现 `width_hold` 试验中，右手 35 条高伸长边有 **27 条边的中心线穿过 pad 实体**，虽然其端点球壳基本未穿入。这直接暴露了原生 PBD 节点接触缺少边/三角面连续碰撞的限制；只筛选节点和绑定前应变，不能声称已经解决边面穿透。

抓取约束是理想无滑动近似。该约束的负载尚未反馈给机器人，也没有验证力闭合、夹持力裕量或力控按压；按压和擦拭动作由位置轨迹实现。机器人模式另通过场景局部适配器补充自由 PBD 节点与刚体接触的库仑摩擦。擦拭时布料保持自由，1 cm 是末端目标行程，布料可能滑动或起皱，实际移动量由 `events.json` 记录；该动作不保证残液全部被吸收。`cotton_contact.py` 在网格采样完成后，仅调整当前场景求解器的碰撞尺寸，保留网格、参考边长、质量与渲染映射。以上扩展均不修改 Genesis 安装源码。

布内输运采用下述有限液体份额近似，不解析棉纤维孔隙、毛细压力、流动惯性、蒸发、挤水、解吸或酱油化学染色。1.5 mm 碰撞壳配合 3 mm 采样间距仍可能漏掉节点之间或折叠处的接触；粒子碰撞不能保证连续三角形接触，快速穿越布面的连续碰撞也尚未实现。当前适用于缓慢动作的介观接触实验，只支持单环境，吸液层不可微分。完整双臂任务由确定性轨迹与液量反馈组成。

## 吸液、携带与状态接口

`coupling.enabled` 默认开启。`cloth_liquid_coupling.py` 在每个原生物理步之后、显示更新之前处理接触、拖曳、吸收与启用的布内输运，`scene.step()` 自动调用这一层。当前双臂配置每步为 2 ms，其他模式以各自配置为准。

接触查询使用变形后物理三角形的精确最近点与重心坐标。只有处于真实布面接触范围、达到连续接触时间，并且当地仍有容量与吸收额度的液体才能转入布内。无接触时不会积攒吸收额度；达到局部容量后停止吸收。

被吸收的 SPH 粒子退出自由液体求解，同时保留独立的质量记录和附着三角形。其质量按重心权重加入原生 PBD 节点，因此湿布的惯性、重力负载和约束逆质量相应改变。附着位置随布变形和布内输运更新，提起后液体仍计入布内质量；布面染深程度由局部饱和度驱动。

当前双臂配置启用 `cotton_wicking.py`：沿参考材料平面的局部饱和度差输运已经吸收的液体，使用未标定的有效系数 **D=5e-6 m²/s**。连续漂移每步不超过 **0.2 mm**，且不超过当前三角形最小高度的 20%。对于连续漂移无法跨出的局部饱和区域，可向**共边邻面**转移一个有限液体份额；其材料位移不超过 **3 mm**，按 `D × dt / d²` 与饱和度差累积通量额度，每份液体每帧最多跳一次。它不向折叠后空间邻近、材料上不相邻的布片转移，也不远程吸收自由液体或增加容量。

输运保留每份液体的质量记录，受接收节点剩余容量限制，并将携带的动量混入接收节点，固定节点的反作用单独记账。更新后的节点湿质量、速度、液体附着三角形及位置同步回原生求解器；检查点同时保存邻面转移的目标与累计额度。该离散过程是有限液体份额的扩散近似，不是解析毛细压力或连续孔隙流，也未验证机械能守恒。

接触拖曳与吸收在各自算子内保持自由布节点和液体的线动量平衡，受约束节点的反作用单独记录。几何防穿透采用独立位置投影。总液体质量始终按“仍然活动的 SPH 质量 + 布内保存的液体质量”核对。

`scene.py` 的 `build_scene(config, ...)` 返回 `(scene, handles)`。常用句柄包括 `handles["robots"]`、`handles["robot_dofs"]`、`handles["cloth"]`、`handles["soy"]`、`handles["coupling"]` 和相机。`handles["coupling"].metrics()` 返回自由液体、吸收量、容量、质量与算子动量残差；`vertex_saturation`、`vertex_absorbed_mass`、`bound_positions` 提供湿度、节点含液量与被携带液体位置。

保存与恢复需要同时保留原生物理状态、吸液及布内输运记录，以及启用机器人抓取时的附着状态：

```python
from scene import capture_coupled_state
checkpoint = capture_coupled_state(scene, handles)
# 继续运行后，恢复同一场景的检查点：
scene.reset(checkpoint)
```

此接口是当前进程内的完整状态对象。单独的 `scene.get_state()` 或输出目录中的诊断 `.npz` 不包含全部耦合信息。`reset_scene(scene, handles)` 用于恢复初始状态与机器人控制目标；抓取中的复位还需要解除附着，具体由场景包装接口处理。

## 可选：受控夹具与静态基线

原有 `--demo-wipe` 保留为吸液耦合验证。它通过规定运动的两条布边夹具扫过酱油并抬起，其余布节点保持柔性，两台 Franka 在这个模式中保持待机。该模式不作为双臂任务的完成证据。

```bash
/home/chloe/Desktop/genesis-local/run-soy-cloth.sh --demo-wipe --record --headless
```

受控夹具模式读取 `config.json`：约 0.20 × 0.14 m、最大 6 mm 厚的液滩，3 mm SPH 粒子；现同样使用 PDF 棉布面密度 0.18 kg/m²、容量 1.08 kg/m²。约 105.45 g 酱油超过整块布约 54.432 g 的容量，该模式不会要求一次完全吸完。输出在 `output/wipe_demo/`，包含 `wipe_fixture_demo.mp4`、`wipe_validation.json`、`mass_history.csv` 和三个阶段的图片。

外层脚本 `run-soy-cloth.sh` **完全不带参数**时默认执行 `--robot-wipe --hold-open`，会运行完整双臂任务。若只需两台机械臂待机的基础场景，直接调用内层入口：

```bash
bash /home/chloe/Desktop/genesis-local/soy_cloth_scene/run.sh
```

静态场景的无窗口检查及流动记录：

```bash
/home/chloe/Desktop/genesis-local/run-soy-cloth.sh --headless --steps 2500 --check-reset
/home/chloe/Desktop/genesis-local/run-soy-cloth.sh --headless --steps 750 --record --flow-probe
```

外层脚本收到参数时会原样转发；上面两条命令没有 `--robot-wipe`，因此运行静态基线。`--flow-probe` 在沉降后施加 0.08 m/s 横向初速度，验证自由液体响应；常规模式的 `--record` 需要搭配 `--steps`。默认输出在 `output/`，独立实验建议指定自己的 `--output` 目录。

## 当前 PDF 材料的运行结果

[CPU 回归日志](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_final_regression.log) 记录 **84 项通过**，包含材料方向、弯曲几何与分组、湿质量参与、阻尼作用范围、吸液账目和状态恢复等检查。[材料移植复核](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_material_review.json) 核对来源与生效参数；它们不替代原生完整动作验证。

后续面积精度修复单独通过 **7 项指标测试**，见干布审计；湿抓反馈控制单独通过 [8 项回归](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_feedback_regression.log)。这些是不同修改范围、不同时间的检查，未将数量相加声称重新跑过完整测试套件。

参考弯曲的同步 Jacobi 累加曾使静置布自发起伏：[旧数值调度静置报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_rest_probe/report.json) 在 0.5 s 时边伸长 P99 为 **12.327%**，不能当作真实材料响应。[独立 CPU 扰动对照](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_bending_diagnostic.json) 在没有重力、接触或机器人的平布上施加 ±1 µm 交错起伏；同样的 B、质量、子步、松弛和四轮求解下，旧算法使起伏标准差增至 **4.978 倍**，17 色互斥求解使其降至 **8.731%**，弯曲能量降至初始值的 **0.5291%**。这验证了该扰动的数值修复，不证明所有非线性状态都稳定。

新 [原生 CUDA 静置报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_rest_colored/report.json) 在 **0.5 s** 时全部节点落在 **Z=0.760750 m**，最高点与最低点一致，最大边伸长为 **4.582×10⁻⁸**（无量纲）；[关闭弯曲的隔离对照](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_rest_no_bending/report.json) 同样保持平布。正式配置仍启用 B=5e-5 N·m 弯曲，不使用关闭弯曲的诊断设置。

[新材料原始完整报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/robot_wipe_validation.json) 已结束：累计 **90.9 s**、**13 个**按压/擦拭周期，1489 份液体、共 **10.48256 g** 全部进入布内，自由液体为 0，液体质量账目误差为 0，原生湿布质量约 **19.55456 g**。但右手最终湿抓没有满足双侧接触条件，未完成湿布提起，报告 **`passed=false`，11/19 项通过**。[本次来源记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/source_manifest.json) 保存只读 PDF、配置和代码哈希，并说明从新干态开始。全部吸液并不等于整项任务成功，下方旧材料视频也不能作为本次结果。

[阶段测量](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/cloth_shape_measurements.json) 中，初抓最大局部伸长 **22.4606%**、全布 P99 **1.5436%**；干布提起末帧的经/纬/斜向最大值为 **2.2493% / 2.2178% / 3.0093%**。经、纬各有一条边超过既定 **2.2%** 阶段上限，未为结果放宽残差。原始完整轨迹采样的最大边伸长为 **37.4715%**，各时刻 P99 的最大值为 **5.8363%**；接触阶段的局部误差仍须保留。

后续 [narrow 试验](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_narrow/robot_wipe_validation.json) 从同一材料 **86.25 s** 拾取前检查点恢复，仅将每指预开口 **4→2 mm**、夹取 TCP **0.770→0.7704 m**，仍在累计 90.9 s 右手湿抓失败，未通过。随后 [feedback 续跑报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/robot_wipe_validation.json) 加入逐步闭合和原门限反馈，在累计 **93.5 s** 完成全部动作：湿布最低点高于桌面 **10.3455 cm**，布内液体上升中位距离 **22.8553 cm**，1489 份液体全部保留、质量误差为 0。绑定时每指保持开度约左手 **1.5018 mm**、右手 **1.4436 mm**；真实接触、材料候选范围和绑定门限没有放宽。

**feedback 为 17/19 项通过，整体仍为 `passed=false`。** 湿布提起末帧经/纬/斜向最大伸长为 **3.8451% / 3.7934% / 5.0318%**，全边 P99 为 **2.0062%**；经、纬分别有 **51 / 13 条**边超过 2.2% 阶段上限。干布和湿布都存在经纬超限，不能说只继承了干布失败。斜向目标为 10%，该方向检查通过。完整轨迹的局部过伸长仍保留，操作完成不是完整材料验收。

可查看 [湿抓及提起续跑视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/dual_arm_wipe.mp4)、[湿布提起全景](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/12_wet_cloth_lifted.png) 和 [湿布近景](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/12_wet_cloth_lifted_detail.png)。视频只记录恢复后的末段；前段来自原始干态运行，不是采用最终控制参数重新完整执行的一轮。[续跑来源记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/source_manifest.json) 记录同材料检查点与控制变化，原 full/narrow 失败报告保留。

为方便查看全动作，另有约 **93.465 s** 的 [动作回顾视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_review/cotton_wipe_review.mp4)：采用原 full 前 1306 帧至约 86.2 s，再接同材料 86.25 s 检查点的 feedback 末段，两个片段均注明中文来源。[视频来源记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_review/provenance.json) 明确它是检查点接续的观看拼接，**不是一次连续完整验证**，也不改变 17/19 项、整体未通过的结论。

[独立末态审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/independent_audit.json) 复核液体保留、容量、表面附着、实际双侧指垫几何接近及原始经纬门限，结论同样未全部通过。17074 条最终布边中心线与作者定义的指垫盒交叉数为 0；同时仍有 3 个自由节点的碰撞壳侵入指垫约 0.09–0.11 mm，不能称为全部接触无穿透。中心线检查不覆盖整个手指网格、三角面内部或连续碰撞，也不证明力闭合。最终逐节点 native mass 与液体重心权重未全部导出，因此完整逐点账目重建在拾取前检查点完成，终态核对保存账目及报告的原生总质量。

[干布独立审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/dry_material_audit.json) 从保存快照复算并确认上述经/纬残差。它另发现初始投影面积的 float32 栅格计算偏差；指标代码已改用 float64，但本次进程载入的 [原指标代码归档](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/source_at_run/cloth_metrics.py) 与原始运行报告均保留。离线复算将初始投影比从 **98.6342%** 修正为 **100%**，干布提起投影比仍约 **44.5993%**。这是测量计算修复，没有重跑或改变物理状态，也没有使经/纬应变检查转为通过；来源与哈希记录在本次 manifest 和审计中。

## 先前材料的验证记录（不适用于当前 PDF 材料）

[独立终态审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/independent_audit.json) 从保存数组复算了液体质量、容量、布面附着和伸长，并核对实际手指姿态：两只夹爪均保留双侧几何接触，18,951 条布边的中心线均未穿入指垫实体。此结论只适用于最终状态，不代表全过程边/面连续碰撞或接触力已经验证。最终原生湿质量与运行报告交叉核对；逐节点原生湿质量及液体到节点的分配重建在拾取前完整检查点独立验证，详见审计中的数据范围说明。

**历史 400 g/m² 材料的验证报告 18/18 项通过，`passed=true`。** [cotton_diagonal_small_pinch 报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/robot_wipe_validation.json) 记录累计 **61.55 s** 仿真、7 个按压/擦拭周期，1489 个液体粒子全部吸收，共 **10.48256 g**，液体质量误差为 **0**。湿布最低点距桌面 **10.89 cm**，布内液体上升中位距离 **24.22 cm**，原生湿布质量约 **30.64 g**。按压后布面及近桌面投影为初始值的 **100.203%**；略高于 100% 来自 1 mm 栅格测量和初始投影基线误差，不代表布料面积增长。

| 提起末帧 | 边伸长 P99 | 最大边伸长 | 超过 5% 的边数 |
| --- | ---: | ---: | ---: |
| 干布 | 0.957681% | 4.879424% | 0 |
| 湿布 | 0.936781% | 4.819909% | 0 |

这 18 项是任务、账目、接触与指定阶段的验收，**不是全过程材料误差均合格**。成功接续轨迹中，采样最大边伸长为 **37.613263%**，各采样时刻 P99 的最大值为 **4.278573%**；初次夹取和铺布仍有局部接触误差。模型仍是未实验标定的各向同性 PBD、节点碰撞和理想无滑移抓取，不能宣称已复现真实棉布全部力学或连续边面接触。

验证前段来自 `cotton_diagonal_complete`，在 **53.85 s** 的 `before_final_pickup.pkl` 保存完整物理与耦合状态，再以修正后的 4 mm 湿抓动作恢复到 `cotton_diagonal_small_pinch`，运行至 61.55 s。[完整验证视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/dual_arm_wipe_verified.mp4) 按这个检查点接续前段和成功末段；[视频来源记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/video_provenance.json) 与 [状态/代码来源记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/source_manifest.json) 说明来源。它不是一次未经中断的录制，也不包含原分支检查点之后的失败尝试；最后 30 帧展示已保存终态，没有继续推进物理仿真。

原 [cotton_diagonal_complete 报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_complete/robot_wipe_validation.json) 在 56.65 s 右手末抓失败，仍为 `passed=false`，只采用其检查点之前的前段作为最终证据。以下三次后续末段试验均完成动作和携带，但湿布最大边伸长超过 5%，也保留为失败历史：

| 末段试验 | 湿布提起最大边伸长 | 结果 |
| --- | ---: | --- |
| [wet_grip](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_wet_grip/robot_wipe_validation.json) | 75.9216% | 材料峰值检查失败 |
| [width_hold](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_width_hold/robot_wipe_validation.json) | 63.0389% | 材料峰值检查失败 |
| [wet_preshape，8 mm](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_wet_preshape/robot_wipe_validation.json) | 11.9188% | 材料峰值检查仍失败 |

原完整运行失败分支的末抓快照最大伸长为 **51.0300%**，64 条边超过 5%；该失败分支不属于成功接续轨迹。另见 [夹取接触审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_grip_contact_audit.md)；其 26.7354% 数值来自更早的短探针，不能与各次完整运行的峰值混用。

历史 **2 ms/40 子步 float32** 完整候选未通过，不能用于材料验证。`cotton_diagonal_final` 在 **23.822 s** 因布料持续上飘而主动中断，[报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_final/robot_wipe_validation.json) 记录吸收量为 0，1489 个液体粒子全部仍自由。原生 PBD 在 `Z≈0.76 m` 处，一个 0.05 ms 子步从静止产生的重力位移约 **2.45e-8 m**，小于 float32 的半 ULP（约 **2.98e-8 m**）；位置舍入后再用 `vel=(pos-ipos)/h` 回算速度，会吞掉这一重力增量。独立 CPU 复现与运行画面均支持该数值问题。

`scene.py` 支持 `simulation.precision`；`cotton_contact_fp64` 的 64 位试运行约需 **70 秒实际时间/1 秒仿真**，已主动取消，未完成验证。最终配置使用可分辨重力增量的 100 微秒子步。

上一轮 `cotton_diagonal_wipe` 已结束并判为失败：[运行报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_wipe/robot_wipe_validation.json) 记录在 **29.7 s** 触发自由液体穿桌/非有限状态检查，仍剩 115 个自由液体粒子，未完成任务。

当时的 [CPU 回归日志](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_final_regression_56.log) 记录 **56 项通过**，包含数值分辨率、湿抓几何和一环应变筛选；仍不能替代原生积分、接触和完整液体任务验证。此前 40 子步短探针的提起读数 P99 **0.4477%**、最大 **1.723%** 受精度问题影响，**撤回其作为低伸长材料验证的意义**，仅保留原始诊断记录。详见 [材料诊断说明](/home/chloe/Desktop/genesis-local/soy_cloth_scene/cotton_material_validation.md)。

历史旧配置的原生 CUDA 双臂流程曾通过 13 项检查，见 [历史运行报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/robot_final_retry/robot_wipe_validation.json)。该运行从 24.276 s 的完整物理检查点修正最终抓取高度后继续，共 30.476 s 仿真、5 次按压，吸收 1489 个液体粒子、10.48256 g 酱油；液体随湿布上升的中位距离为 24.36 cm，湿布最低点距桌面 18.84 cm，液体质量误差为 0。旧参数与当前棉布材料、抓点和擦拭动作不同，**这 13 项通过不构成当前配置的验证**。

[历史完整视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/dual_arm_wipe_verified.mp4) 将检查点前后的原始录制按同一物理状态接续。`robot_probe*`、`output/robot_wipe/` 和 `output/robot_live/` 保留历史调试记录，各自结果仅适用于其保存的配置，不表示当前任务正在运行或已经通过。

- 小液滩的 [CPU 局部容量与通量验算](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/robot_profile_feasibility.json)：使用与原生规则采样对应的 1489 个液体粒子、1543 个重新划分的布节点。在静止布面持续接触条件下，10.48 g 液体于 3.236 s 内全部转入布内，最高局部饱和度约 76.25%，液体质量账目误差为 0。这是耦合算子验算，未运行原生液体动力学或机器人动作。
- 原生 CUDA [受控夹具报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/wipe_demo/wipe_validation.json)：12 秒扫过/提起测试通过；105.45 g 初始液体中约 34.00 g 转入布内，湿布质量由约 11.09 g 增至 45.09 g，被携带液体的中位上升高度约 11.12 cm，完整复位通过。此测试约束 55 个布边节点。
- [原生边界检查](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/coupling_edge_checks/validation.json)：测试专用高容量、高通量条件下，全部液体吸收后的空液体渲染、湿状态检查点和干状态复位通过。其参数只用于边界检查，未用于双臂任务材料配置。
- 吸液扩展前的 [流动检查](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/flow_check/final_validation.json)：0.08 m/s 横向初速度使液体质心沿 X 移动约 3.57 mm；另有 2500 步、10 秒静态稳定性与复位基线。它们验证基础场景，不验证抓取与吸液任务。

模型回归测试可重跑：

```bash
/home/chloe/Desktop/genesis-local/.venv/bin/python -m unittest discover -s /home/chloe/Desktop/genesis-local/soy_cloth_scene/tests -v
```

单独重跑小液滩容量/通量的 CPU 验算：

```bash
/home/chloe/Desktop/genesis-local/.venv/bin/python /home/chloe/Desktop/genesis-local/soy_cloth_scene/check_robot_profile.py
```

该验算不创建 Genesis 场景，也不占用 GPU。其通过不能替代完整双臂动作、实际接触、自由液体残量与最终提起的运行检查。
