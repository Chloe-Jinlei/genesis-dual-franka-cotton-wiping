# 双 Franka 抓布、吸液与提起任务

本目录使用本地 Genesis，包含桌子、两台 Franka Panda、长方形柔性棉布和可流动的 SPH 酱油。`robot_wipe.py` 使用两台机械臂抓住布料的两个对角，执行搬运、铺布、松开、上下按压、同步平移擦拭、重新抓取和提起湿布；吸液量由局部接触、吸收通量和有限容量决定。当前采用用户 PDF 所列的二维机织棉布近似，材料参数尚未实验标定。

当前材料以用户 PDF 的 `plush_cotton` 参数为基础：180 g/m²、经/纬/斜向 2%/2%/10% 伸长投影目标、B=5×10⁻⁵ N·m 二面角弯曲，使用保留经纬的规则物理网格。来源是只读的 [用户 PDF](/home/chloe/Desktop/熊与棉布_物理参数设置_plush_cotton.pdf) 第 2、4、6 页及对应本地实现。此前为提起后的持续振动加入双精度、保留积分速度的重建、自由布料变形残差阻尼和干抓反馈闭合；其中新增耗散模型是工程近似，不能称为 PDF 已标定物性。

**当前默认控制器的完整任务已运行完成：张开下降 → 实测接触布角 → 逐步夹紧 → 对角提布、铺布 → 六轮擦拭、全部吸液 → 最终夹取并提起湿布。** [本轮完整中文视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/continuation_review/cotton_contact_first_full_cn.mp4) 包含 87.05 秒动作和额外 4 秒保持。10.48256 g 酱油全部吸收，16 项任务要求通过；严格经纬材料检查仍未通过。最终四指垫在完成及保持后均满足额外的每指至少 2 节点检查。中途因用户中断，于 38.7 秒完整物理检查点原状态恢复，属于相同控制、相同物理设置的两段接续，非一次不中断运行。详见本文末尾“2026-11-12 完整仿真”。

**此前版本已完成完整任务，严格材料方向检查仍有警告。** [最终原生报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_tail_wipe/run/robot_wipe_validation.json) 在累计 **97.95 s、8 个按压/擦拭周期**完成搬运、铺布、完全吸液和湿布提起：`task_passed=true`，16 项任务要求全部通过；原始全部检查仍是 **17/19、`passed=false`、`strict_material_passed=false`**，经向和纬向超限没有被豁免为材料合格。1489 份酱油共 **10.48256 g** 全部进入布内，自由液体为 0，原生湿布质量 **19.55456 g**。用户认可的棉布材料、精度及耗散设置保持不变。

完成证据是 **五段相同物理设置的原生检查点接续**，不是最终控制配置从新干态一次不中断运行；来源和各段边界见下文。[独立终态审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_tail_wipe/run/independent_audit.json) 的 45 项保存数据核查全部通过，严格经/纬检查仍失败。该次控制曾提升为默认，见 [默认提升记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/default_promotion.json)。历史 float32 PDF 的 17/19 和更早 400 g/m² 的 18/18 是不同版本，原报告和失败结论继续保留。

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

`--robot-wipe` 自动读取 `config_robot_wipe.json`。这个模式的 `--record` 不需要提供 `--steps`；提供 `--steps` 会限制总步数，提前停止会记录为未完成。首次运行需要编译计算内核，实际运行耗时通常长于视频中的仿真时间。当前物理精度为 float64，计算代价明显高于先前 float32；本机稳定推进耗时为 **约 48.29 秒实际时间／秒仿真（本机 NVIDIA GeForce RTX 5070 Laptop GPU，[原生运行日志](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_settle_fp64.log)中 9.5–13.0 s 静止保持段实际用时 169.0 s；含录制与控制开销，不含初始化和内核编译）**。历史 70 倍耗时来自不同旧配置，不能直接当作本轮测量。

当前默认控制已取消空中预闭合；完整新控制器运行及四次抓取审计见本文末尾。擦拭路径如下；每次新运行仍须以自己的原生报告验收：

1. 双臂从每指 **10 mm** 开口开始（指端净间隙约 **20 mm**），保持张开接近布料的两个对角。TCP 先到 **0.7704 m**，若完整手指几何尚未接触布料碰撞壳，则保持开口以 **0.2 mm** 小步下降，最低目标 **0.7698 m**。只有实际 TCP 距目标不超过 **0.6 mm**，且每手至少一根机械指的完整原生碰撞几何与原布角接触，才允许开始闭合。使用与布料碰撞相同的 SDF，阈值是实际碰撞半径加 **50 µm**；指垫距离仅作诊断。该几何到位检查不是力闭合证明。左/右参考抓点为布中心加 `(-0.13,+0.08)` / `(+0.13,-0.08)` m。
2. 到位后干抓和湿抓才开始反馈闭合：每指大于 **2 mm** 时每步最多收 **1 mm**，到 **2 mm** 后使用原 **0.1 mm** 小步；每步均为 **0.05 s 移动+0.05 s 等待**，逐步检查原抓取门限。较宽开口的闭合会真实扫动布料，不能视作无接触空程。干抓受挤失败时，只让未成功的手停在实测指宽、上移 1 mm 释放挤压，再检查原门限；成功手保持，正式提起仍需双手通过。两侧指垫各须至少两个真实接触节点；只从原材料角部 **22 mm** 范围各选一个不同代表，每手固定两点。各手一旦通过接触、材料身份、拓扑、非侵入和局部应变门限，即保持其实测开度；另一手可以继续闭合。门限不因重试而放宽。
3. 共同提起棉布，搬运到酱油上方并下降铺布。解除抓取后，双手夹指从绑定保持 **10000/100** 恢复为正常 **2000/40** 增益，再张开和按压。
4. 初始完整周期沿世界 **−Y 3 cm** 擦出并返回，每段 **1.6 s**；出程额外停留 **0.75 s**，原有每次移动末尾 0.20 s 稳定时间仍保留。布料已解除抓取，由实际接触摩擦带动，液体仍按有限局部容量吸收和在布内输运。
5. 完整第 **2 轮**后若仍有液体，真实重新夹取同一原材料对角，抬至 TCP **0.96 m**，将布中心移至酱油中心加 `[-0.06,-0.045]` m，即 `(-0.06,-0.005)` m，铺下、释放并恢复正常增益；下一轮改为世界 **+Y 1 cm** 往返。完整第 **3 轮**后，第二次真实提布换区至酱油中心加 `[-0.06,0]` m，即 `(-0.06,0.04)` m，后续改为世界 **+Y 3 cm** 往返。
6. 从已完成第 3 轮后的湿抓开始，对原参考材料点与实际平布 XY 作无缩放刚体拟合，观测中心和 yaw。空夹爪先在原布局升至安全高度 **0.80 m** 再共同对齐，双手偏移及手姿一起旋转，保持跨度；后续抓、抬、移、铺、按压和最终提起均沿用观测布局。原材料候选身份不变，换区和擦动仍是世界 XY 方向。布面不够平、拟合误差过大或真实抓取门限失败时停止。
7. 完整第 **7 轮**后仍有液体时，显式行程计划将往返改为世界 **+Y 4.5 cm**，中心/yaw/原抓点和时间设置保持。计划只在周期边界应用一次，并保存事件及恢复历史；不移动粒子、不借远处干区容量，也不增加 14 轮上限。旧版曾在第 8 轮后归零；本轮在第 6 轮归零，未触发该额外行程。
8. 全部吸液且至少完成两个周期后，再次观测实际布面、反馈闭合夹住同一材料角块，用 **3.5 s** 提起湿布、稳定 **1 s**。湿抓同样保持每指 10 mm 开口下降，在原布角接触前不闭合。下探在张开状态完成；接触后闭合若仍未夹住，可保留原 1 mm 上抬缓解动作，再检查原门限。任务完成后独立报告整布方向应变。

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
| `before_pressing.pkl` / `after_press_cycle_N.pkl` | 原生动力学、吸液账本和累计控制状态；轮数仅计完整周期 |
| `after_reposition_N.pkl` / `after_wipe_stroke_N.pkl` | 成功换区或行程选择后的状态，包含中心/yaw、已执行计划及历史 |
| `before_final_pickup.pkl` | 全部吸液后的实际湿抓前状态 |
| `after_wet_lift.pkl` | 终态原生逐点质量和液体分配审计；stage 为 `audit_only_complete`，普通恢复入口拒绝继续 |

`passed: true` 仍只表示全部原检查通过，原值不会因用户接受材料外观而改写。PDF 方向模式另提供 `task_passed` 和 `strict_material_passed`：前者要求全部 16 项动作、质量、容量、接触、覆盖、碰撞、跟踪及绑定长度条件通过；只有三项干/湿提起方向峰检查可单列为 `material_warnings`。缺失/非法检查、运行错误或未知失败仍阻断任务。严格材料字段只覆盖这些阶段方向检查，不代表全过程材料认证。

当且仅当任务通过、仅有方向材料警告时，程序明确提示警告并允许 `--hold-open` 保留窗口；其他失败仍停止。湿布整体离桌与吸收液体上升均须超过 6 cm。视频本身不替代这些检查。

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
| 张开接近 | 初抓、换区、最后湿抓均每指 10 mm，指端净间隙约 20 mm；更宽的现有开口不会在空中缩小。湿抓先在安全高度 0.80 m 对齐 |
| 指关节 PD 增益 | 初抓和湿抓闭合均为 2000/40；绑定保持 10000/100；解除抓取后恢复 2000/40 |
| 首次 / 最终夹取 TCP 高度 | 从 0.7704 m 开始；保持张开，最多三次 0.2 mm 下探，最低目标 0.7698 m；真实到位且每手至少一根完整机械指与原布角的原生碰撞壳接触后才闭合 |
| 干抓 / 湿抓反馈闭合 | `dry_grasp_feedback_closure=true`、`wet_grasp_feedback_closure=true`；每指大于 2 mm 时最多 1 mm 步长，之后 0.1 mm；0.05 s 移动加 0.05 s 等待，通过严格门限即保持实测开度 |
| 平移擦拭 | 初始 −Y3cm；两次换区后 +Y1cm、+Y3cm；第7轮后 +Y4.5cm。出/回各1.6s，出程额外停留0.75s；世界方向不随布局yaw旋转 |
| 每只手的接触 / 固定节点 | 每侧指垫至少两个真实接触节点；每侧固定一个不同代表，共两个固定节点 |
| 固定点对伸长上限 | 2%；仅约束建立抓取时选中的固定点对，不代表整块布的所有边始终低于 2% |
| 干抓和湿抓绑定前几何 | 均启用 `preserve_contact_geometry`：候选球壳侵入 pad ≤50 µm；直接固定邻边为 ±2%，候选一环真实网格边为 ±5% |
| 湿布提起 | 3.5 s 提起，随后稳定 1 s |
| 伸展约束 | `stretch_compliance=1e-7 m/N`、松弛 0.3、每子步 10 轮；随后 24 轮经/纬/斜向 2%/2%/10% 伸长投影 |
| 弯曲约束 | `B=5e-5 N·m` 的几何加权二面角模型，松弛 0.5、4 轮；每轮按 17 个互斥四节点组顺序求解，每子步清零乘子 |
| 空气阻力 | 各向同性二次阻力；空气密度 1.225 kg/m³、阻力系数 1.0，各节点按相邻参考三角形面积的三分之一求和 |
| 棉布内部阻尼 | `internal_damping_model="rigid_motion_residual"`，4 s⁻¹；质量加权拟合自由布料刚体运动，仅指数衰减变形残差，替换旧逐边轴向阻尼 |
| 局部弯曲速度阻尼 | `bending_damping_ratio=0.5`；按二面角速度作隐式耗散，局部比值不是整布的模态阻尼比 |
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
| 数值精度 | `simulation.precision="64"`，刚体、PBD、SPH 使用原生双精度；布料世界坐标 Z 上限仍为 1.25 m |
| PBD 速度重建 | `preserve_integrated_velocity=true`；保留力积分后的预测速度，再叠加约束位置修正除以子步时长 |

规则采样的小液滩包含 1489 个液体粒子、约 10.48 g 酱油。历史 CPU 验算在平展、持续接触的布面上验证过容量与通量；这不能保证实际折叠、铺布后的局部材料容量足够。实际运行的粒子数、局部饱和度和残液质量以该次报告为准。

容量依据每个节点的参考面积分配，不能从远处未接触的干布区域借用容量。6 g/g、吸收通量 1 kg/(m²·s) 和输运系数 D=5e-6 m²/s 都是独立、未标定的吸液假设，**不是 PDF 提供的材料数据**。新布面积为 0.0504 m²，按 180 g/m² 得到干质量 9.072 g、总容量 54.432 g；原熊场景布料的 29.39 g 来自不同尺寸，不能直接照搬。网格采样间距约 3 mm，碰撞壳总厚度独立设为 1.5 mm；这是接触近似，不是实测布厚。

当前保留原生边伸展求解，再对初始经、纬、斜向网格边施加不同的拉伸投影目标。2%/2%/10% 是有限轮数约束的目标，不是任何时刻都满足的硬限，也不是实测断裂应变；后续弯曲、碰撞和抓取仍会留下超限残差。方向分组提供机织布的简化响应，尚未包含实测非线性应力–应变、纱线卷曲或独立剪切本构。报告按方向列出实际最大伸长与超限量；干/湿提起阶段的方向验收另允许 0.2 个百分点数值残差，不能用旧统一 5% 上限替代。

布料可下垂、起皱和折叠，这些形状变化本身不等于材料被拉长；伸长始终相对初始物理边长计算。斜向 10% 目标作用于网格对角边，为斜向形变保留空间，并非所有方向采用相同的弹性伸长。夹爪附近的过伸长记录仍是需要核查的离散约束/接触残差，不能解释为真实棉布的橡胶式弹性。

空气阻力的每节点系数为 `0.5 × ρ_air × C_d × A_i`，`A_i` 是该节点相邻参考三角形面积之和的三分之一，所有 `A_i` 的和等于布料参考总面积。它不是将面积均分为 `A/N`，也不改变原生均匀干节点质量。此处理避免网格加密放大总阻力；仍是各向同性阻力近似，没有解析布面朝向或透气性。内部阻尼现在对自由且活跃的布节点按当前湿质量拟合整体平移和转动，只将剩余变形速度乘 `exp(-4h)`；固定节点排除在拟合之外且速度不改，整布刚体运动保留。这是非局部耗散近似，替换此前的逐边轴向 4 s⁻¹ 模型，并不等价于 PDF 的已标定黏弹性。另保留局部二面角速度阻尼 ζ=0.5 与棉布线性阻力 1 s⁻¹。各算子读取当前原生湿质量；局部被动性不能证明接触、附件与约束的整个分裂求解不输入能量。

与参考场景相比，当前还保留以下数值设置：参考子步为 1 ms，这里沿用适合现有 SPH 耦合与位置分辨率的 **100 µs**；碰撞直径沿用 **1.5 mm**；参考全场 **1 s⁻¹** 线性阻力限定为只作用棉布，保持酱油动力学参数不变。此外，参考实现的弯曲 Jacobi 同步累加在细网格上会放大交错扰动，现改为四节点互斥分组的 Gauss–Seidel 顺序求解；弯曲能量公式、B=5e-5 N·m、松弛 0.5 和 4 轮均保留。既有调度对照见下方历史材料验证；本轮新增双精度、速度重建及耗散模型的范围见“当前双精度稳定性修正与验证范围”。

`numerical_resolution.py` 在构建前要求 `9.81 × h² > 0.5 × ULP`，ULP 按当前精度及最大布料世界坐标 Z=1.25 m 计算。历史 float32 的 `ratio≈0.823` 不是当前 float64 的分辨率；报告应记录本次实际 dtype 和重算比值。该门限只排除重力位移被完全舍入。另启用的速度重建采用 `v=v_pred+(x_projected-x_pred)/h`，精确算术下与原式等价，避免小的积分增量先因绝对位置差而丢失；它不修复位置投影自身或后续碰撞的数值误差。

## 抓取、按压、擦拭与模型边界

机器人关节通过 Genesis 的 PD 动力学控制，末端路径由逆运动学求解。接触中允许不超过 5 mm 的末端跟踪误差，刚体碰撞另外检查。最低目标 TCP 高度 0.7698 m 在标称姿态下保留约 0.442 mm 指尖与桌面间距，实际接触仍须检查真实链接位置。`robot_grasp.py` 只在指定的初始材料角块中寻找实际接触节点，不借用恰好折叠到附近的其他布料。节点须位于真实夹爪之间且距当前 TCP 不超过 12 mm，两侧指垫各至少接触两个节点；接触距离上限仍为碰撞壳半径加 0.3 mm，即 **1.05 mm**。

当前干抓和湿抓均启用 `preserve_contact_geometry`，使用相同的 **7.5 mm 中央 pad 间距上限**：Panda 中央 pad 内面间距比更靠下的小 pad 间距大 3 mm，因而小 pad 上限 4.5 mm 对应中央上限 7.5 mm。此前干抓沿用中央 4.5 mm 的旧门限，本轮将干抓切到已有湿抓严格模式，是明确的策略变化。真实双侧支持、材料身份、TCP 距离和拓扑要求均继续检查。

两侧接触集合必须通过局部真实网格路径连通。当前连通区域按 25 mm 加碰撞壳半径检查，节点中心距 TCP 上限为 **25.75 mm**，最短布面路径不超过 **51.5 mm**。每侧至少两个真实接触节点的验证通过后，当前配置从各侧选一个不同的代表，形成 **1+1 个固定节点**；两代表之间也必须满足相同的真实网格路径限制。固定点对当前距离不得超过初始材料距离的 1.02 倍，避免锁住已被拉长的材料。没有合格子集时停止并报告，不放宽接触门限。报告分别保留原双侧接触证据、代表节点的连接路径、初始材料包围盒及固定点对伸长。

满足全部条件后，选中节点按当时的局部偏移约束到真实手部链接；建立约束不会把布传送到手中。连通路径中间节点和未选中的真实接触节点保持自由，由原生接触和场景摩擦模型作用，减少角部过约束。2% 检查针对建立约束的固定点对，整块布的局部伸长另由 `cloth_shape_measurements.json` 和阶段 `.npz` 检查。接口的 `minimum_bound_per_finger` 默认仍为 2，当前双臂配置显式传入 1。

当前干抓和湿抓筛选均要求候选球壳对任一 pad 的侵入不超过 **50 µm**；会触发原生自碰撞的固定点对，其球壳重叠也不得超过该容差。直接连接两个固定点的网格边必须处于参考长度的 **98%–102%**，选中节点一环内所有真实网格边必须处于 **95%–105%**。这避免绑定已经不兼容的局部几何，但不保证之后的接触不会再次卡住。各手绑定后保持当时测得的指关节开度，使用 PD 10000/100，避免继续向零闭合挤压固定节点。当前仍绑定到 `hand`；`finger_pads` 绑定选项仅做过 CPU 测试，生产配置不使用它。

历史 400 g/m² 材料的 [初次夹取接触审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_grip_contact_audit.md) 发现短探针夹取瞬间有 31 条自由节点间的边超过 5%，峰值 **26.7354%**，并存在约 **0.6463 mm** 的局部碰撞壳压入桌面残差。证据支持指尖底部空间不足与顺序接触投影造成局部网格畸变，但尚无修改几何后的对照仿真证明。该问题没有被低伸长抓取门限完全消除；任务及干/湿布提起检查通过，也不能表述为全过程所有边均低于 5%。

同一旧材料的 [湿布夹持审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_wet_grip_conflict_audit.md) 还发现 `width_hold` 试验中，右手 35 条高伸长边有 **27 条边的中心线穿过 pad 实体**，虽然其端点球壳基本未穿入。这直接暴露了原生 PBD 节点接触缺少边/三角面连续碰撞的限制；只筛选节点和绑定前应变，不能声称已经解决边面穿透。

抓取约束是理想无滑动近似。该约束的负载尚未反馈给机器人，也没有验证力闭合、夹持力裕量或力控按压；按压和擦拭动作由位置轨迹实现。机器人模式另通过场景局部适配器补充自由 PBD 节点与刚体接触的库仑摩擦。擦拭时布料保持自由，各阶段的 1–4.5 cm 是末端目标行程，布料可能滑动或起皱，实际移动量由 `events.json` 记录；该动作不保证残液全部被吸收。`cotton_contact.py` 在网格采样完成后，仅调整当前场景求解器的碰撞尺寸，保留网格、参考边长、质量与渲染映射。以上扩展均不修改 Genesis 安装源码。

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

## 当前双精度稳定性修正与验证范围

以下记录用户认可材料之前的稳定性修正与独立探针；当时的默认提升、源码哈希和验证范围见 [默认配置及验证汇总](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_settling_fix/final_validation.json)。

该稳定性修正阶段保持 PDF 的干面密度、伸展顺应度、经纬/斜向目标、弯曲刚度和摩擦参数，改变的是数值精度、速度更新、干抓控制及耗散模型。4 s⁻¹ 的数值沿用 PDF 内阻尼设置，但全布自由集残差模型是新工程选择；局部二面角阻尼 ζ=0.5 也是新增假设。二者均未用布样实验标定，不应描述为“完整复现 PDF 材料”。机制及局限见 [本轮研发说明](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_settling_fix/mechanism_notes.md)。

| 验证范围 | 本轮结果与来源 | 验证边界 |
| --- | --- | --- |
| CPU 回归与算子守恒 | **[CPU 回归日志](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_settling_fix_default_regression.log)：129 项通过，7.216 s** | 不覆盖原生接触和整套机器人流程 |
| 新干态实际双臂抓取、提起、静止保持 | **[干抓保持验收](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_fp64_review/settling_validation.json)：保持 4 s；末窗口 3≤t<4 s 的节点运动范围 P95 为 0.10386666 mm，平均自由节点速度 RMS 为 0.1496455 mm/s，TCP 窗口内最大两两位移 ≤2.34×10⁻¹⁵ m；稳定性通过，纬向材料检查失败。推进约 48.29 秒实际时间／秒仿真** | 不覆盖湿抓、吸液和完整擦拭 |
| 0.5 s 原生双精度短时吸液 | **[短时吸液报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_wet_precision/report.json)：0.5 s、8 项通过；吸收 227 份、1.59808 g，仍有 1262 份自由液体；原生位置/速度/质量均 float64，湿布 10.67008 g，湿质量差 −4.34×10⁻¹⁹ kg，附着误差 0；阻尼读取湿质量检查通过** | 尚有自由液体，不覆盖真实擦拭和湿布提起 |
| 后续同材料完整动作链 | **已由下节五段原生检查点接续完成；任务通过，严格经纬检查未通过** | 最终控制配置未从新干态不中断重跑，两个旧探针也不作为完整动作证据 |

[流程前后对照](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_fp64_review/robot_comparison.json)中，旧流程保持末秒的节点运动范围 P95 为 **21.02428 mm**，新版为 **0.10386666 mm**；旧版从原干布提起检查点继续，新版从干态重新抓取，代表节点、精度、速度更新、阻尼和抓取控制都有变化。这是实际流程前后对照，**不是同一物理状态的单变量实验**，不能将差异全部归因于某一个参数。可查看 [新版实际抓取与保持视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_fp64_review/new_robot_full.mp4) 和 [保持阶段前后对照视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_fp64_review/robot_hold_before_after.mp4)；两段视频的范围是干抓、提起和保持，不是完整酱油擦拭。

稳定性通过不覆盖材料方向验收：提起末帧的纬向最大伸长为 **2.278154%**，保持结束为 **2.267258%**，两阶段均有 **2 条纬向边超过既定 2.2% 上限**；经向和斜向通过各自阈值。因此 `settling_passed=true`，但 `material_direction_checks_passed=false`、合并结果仍为 false；没有放宽阈值，也没有宣称全部材料验收通过。短时吸液的液体总质量误差为 **1.73×10⁻¹⁸ kg**，证明本次局部精度与质量通路一致，不代表全部液体已经吸收。

短时吸液探针把布在初始化时放到原液滩上方，再由原生重力和普通耦合运行；它不执行机器人擦拭，也不在构建后人为搬动粒子。吸液公共 setter 已去掉六处中途 float32 量化，按 `gs.tc_float` 转换；湿质量字段保留原生 dtype。湿色显示仍用 float32 颜色缓冲，不回写物理。`cotton_robot_settle_fp64` 干抓进程启动时载入的旧耦合源码另存于 `source_at_run/`，其 [补充来源记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_settle_fp64/extra_source_manifest.json) 与之后真正吸液探针的源码记录须分别引用。

精度和材料模型改变后，旧 float32 检查点不能直接恢复为本版本；恢复前须通过配置/模型签名与原生数组 dtype 检查。新版本从干态建立自己的状态来源。完整方向验收仍保留经/纬/斜向 2%/2%/10% 目标及另列的 0.2 个百分点数值残差，不因振动减小而放宽。

## 当前双精度任务完成记录与来源

[最终原生报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_tail_wipe/run/robot_wipe_validation.json) 的 `failure=null`、`task_passed=true`，16 项任务要求全通过；`passed=false`、`strict_material_passed=false`，仅经/纬两项方向检查失败。完整动作和吸液携带已经完成，不能写成全部材料验收通过。

| 本轮结果 | 实测值 |
| --- | ---: |
| 累计仿真 / 完整擦拭周期 | 97.95 s / 8 |
| 自由液体 / 已吸收 | 0 份 / 1489 份，10.48256 g |
| 原生干布 / 湿布质量 | 9.072 g / 19.55456 g |
| 湿布最低点 Z / 离桌 | 0.871018658 m / 11.101866 cm |
| 吸收液体上升中位值 | 23.270457 cm |
| 全程采样质量误差最大值 | 3.46945×10⁻¹⁸ kg |
| 末端跟踪误差最大值 / 原生刚体碰撞样本 | 0.430535 mm / 0 |
| 初次铺布投影 / 近桌投影比 | 均为 99.064116%，按原报告 1 mm 栅格口径 |

[独立终态审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_tail_wipe/run/independent_audit.json) 确认 45 项保存数据检查全部通过、无未知项，并保留 16 项原任务检查。终态检查点直接导出 native 逐节点湿质量和每份液体的三角面/重心权重，逐节点增重最大误差约 **8.47×10⁻²² kg**；1489 份液体均保留并随布携带。两手终态各指垫接近节点分别为 **2/2、3/2**，所选代表仍在原材料角部；17074 条边中心线未穿过作者定义的 pad 盒。仍有自由节点球壳局部侵入 pad 约 **0.105–0.110 mm**，这些检查不覆盖整个手指实体、三角面连续碰撞或力闭合。

| 提起末帧 | 经向最大值 / >2.2% 条数 | 纬向最大值 / >2.2% 条数 | 斜向最大值 / >10.2% 条数 | 全边 P99 |
| --- | ---: | ---: | ---: | ---: |
| 干布 | 2.063263% / 0 | 2.221524% / 2 | 1.853806% / 0 | 1.099707% |
| 湿布 | 2.518809% / 14 | 3.063012% / 7 | 3.658817% / 0 | 1.782723% |

2%/2%/10% 投影目标及 0.2 个百分点阶段残差均未更改。50 ms 间隔采样的整个接续轨迹最大局部边伸长为 **55.485411%**，各采样时刻 P99 的最大值为 **4.010139%**；初抓、初铺、两次换区铺下的阶段最大值分别为 **5.827114%、7.525228%、12.338906%、11.197673%**。这些局部投影/接触残差没有因用户认可当前布料而被删除，不能解释为真实棉布可作相同弹性伸长，也不能声称全过程材料严格合格。

本轮五段原生状态接续如下。所有段保持已认可的材料、精度、耗散、液体和吸液耦合；变更的是释放控制、擦动/换区、布局观测和结果分类。

| 原生来源 | 采用区间或边界 |
| --- | --- |
| `output/cotton_fp64_task` | 新干态至 `after_dry_lift.pkl`，9.45 s |
| `output/cotton_fp64_wide_wipe/run` | 9.45→27.90 s，至完整第2轮检查点 |
| `output/cotton_fp64_reposition/retry_run` | 27.90→44.85 s，第一次换区及完整第3轮 |
| `output/cotton_fp64_layout_tracking/run` | 44.85→82.45 s，实际布局对齐、第二换区及完整第7轮 |
| `output/cotton_fp64_tail_wipe/run` | 82.45→97.95 s，+Y4.5cm末轮、最终湿抓及提起 |

各段的检查点哈希、事件/CSV 前缀和材料一致性已在独立审计复核。原 layout 运行到 83.04 s 后停止的部分第8轮，以及之前异常终止或抓取失败的分支均排除。最终湿抓实际重新测得 yaw **+9.601797°**，不是直接沿用离线假设的 +6.352424°。动作来源是五段接续，**不是最终配置一次不中断的新干态验证**。[完整动作回顾视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/cotton_wipe_complete.mp4) 已完成并核验：1484 个原生帧、97.9538 s，五段边界无重帧/漏帧、无变速或停帧；[视频来源记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/composition_provenance.json) 和 [逐帧来源表](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/source_frame_map.csv) 可复核接续。末段 1 s 保持的 16 帧未见明显大幅反复弹跳，仍有细微自由角变化；同期 21 个 CSV 样本最低布高峰峰值为 1.182 mm。这仅是本次 1 s 观察，不能替代此前 4 s 稳定性指标。

该次完成后，默认 `config_robot_wipe.json` 曾与成功末段候选一致，SHA-256 为 `c94b8b453a9402a8b9b39dab65347e2b80216c8ddab14936fcd5dea79964cce3`；[提升记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/default_promotion.json) 证明当时仅提升 `robot_task`，材料配置未改。该候选曾通过 176 项 CPU 回归；默认提升后，旧基线测试被新增可选计划影响，失败日志保留。只修复测试夹具隔离并加入默认配置集成检查后，[当时最终 CPU 回归](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/default_regression_after_fixture_fix.log) **177/177 通过，8.000 s**；生产实现、物理参数与验收门限没有为测试改变。此 SHA 和测试数量属于历史版本，当前抓取修正见文末。

## 历史 PDF 材料运行结果（float32，不是本轮完整回归）

以下原始报告、84 项旧回归及 17/19 末段续跑均来自此前 PDF 材料版本；保留其数值与失败结论，不覆盖为本轮结果。

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

当时 `scene.py` 已支持 `simulation.precision`；历史 `cotton_contact_fp64` 的 64 位试运行约需 **70 秒实际时间/1 秒仿真**，已主动取消，未完成验证。该轮最终采用 float32 和 100 微秒子步。此旧耗时与取消记录不代表本轮双精度候选的耗时或结论。

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

## 2026-11-11 夹爪提前闭合修正

旧控制在布料上方就将每指开口收到 2 mm，然后才下降，正是截图中“没碰到布就合上”的原因。默认配置现在从每指 10 mm 开始，下降期间不缩小开口；用原生完整手指碰撞几何测量原布角接触，闭合前立即复查实际到位。接触后才逐步闭合，正式提起仍须通过原来的双侧指垫、原材料身份、拓扑、非侵入与局部应变门限。初次干抓、两次换区湿抓和最后湿抓共用该顺序。

配置改动仅为初始开口、干/湿接近开口和启用接触后闭合的开关，[配置差异](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/config_change_review.json) 可复核。当前配置 SHA-256 是 `b68b90539ed075997d9d1f312758058ed26c8f57ba2c66ff2da21009bd127d0e`；棉布、时间步、阻尼、酱油和吸液设置未改。

本次两段原生运行分别验证新干布首抓，以及从旧完整任务真实 `before_final_pickup.pkl` 状态继续的最后湿抓；两段没有拼成“整轮重新运行”。湿抓前的 88.6 s 历史仅作为已吸液状态来源，不算新控制器验证。

| 原生动作 | 保持开口抵达布角、随后开始闭合 | 双手夹持确认 | 主提起开始 | 提起完成 |
| --- | ---: | ---: | ---: | ---: |
| 新干布首抓 | 5.55 s | 6.85 s | 7.05 s | 9.25 s |
| 已吸液湿布末抓 | 91.95 s | 93.55 s | 93.75 s | 98.45 s |

两次均在原标称 TCP 高度 0.7704 m 测得每手至少一根完整机械指与原布角碰撞壳接触，之后才开始闭合；没有用“接近目标位置”替代接触，也没有使用额外下探或失败后的缓解分支。双指支持是在闭合过程中另行确认的。每段五项动作顺序检查全部通过，见 [干抓顺序记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/dry_native_contact_03_review/contact_first_review.json) 和 [湿抓顺序记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/wet_native_contact_01_review/contact_first_review.json)。

可观看 [干布张开接近、夹紧和提起](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/dry_native_contact_03_review/dry_contact_first_native.mp4) 与 [湿布接触后夹紧、提起](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/wet_native_contact_01_review/wet_contact_first_native.mp4)。中文标注视频分别来自单段 147/157 个原生帧，未变速、补帧或拼入失败试验。两段额外 0.5 s 保持未见明显大幅反复弹跳，最低布高峰峰值分别约 1.060/0.563 mm；这不是 4 s 稳定性验收。

[本次 CPU 回归](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/final_cpu_tests.log) 196/196 通过。[离线路径复核](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/path_clearance_final_summary.json) 检查至 14 轮、两次换区和最深允许张开下探，未检出规定几何碰撞；该复核假设接触和绑定，不能替代原生抓取或整轮擦拭。原生干抓达到提起阶段；原生湿抓达到最终提起阶段，1489 份、10.48256 g 液体保留在布内，原生湿布质量 19.55456 g，最终最低布高距桌面约 10.922 cm。

额外的 [独立湿抓审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/wet_native_contact_01/independent_audit_summary.json) 没有全部通过：提起后的右手一个指垫只有 1 个节点处于原接近门限内，未达到独立终态复核的每指 2 节点要求。各指固定代表仍有支持；闭合/主提起前的原双侧门限确实通过，不能把该终态问题改写为抓前通过即全程通过。严格材料残差也继续保留：湿布完成提起时经/纬最大伸长约 2.453%/2.337%，仍高于 2.2% 阶段上限。本次结论是提前闭合问题已修正并验证，**不是全部终态接触或材料检查均合格，也不是整轮新控制器擦拭已经重跑**。

早期 `dry_01`、`dry_relief_02` 的失败分支原样保存；它们使用过于严格的仅指垫到位判断而过度下探，不作为成功视频或当前接触判断的证据。后续采用与原生布料碰撞相同的完整手指 SDF，只读查询状态，不移动布粒子，不放宽原抓取门限。

[本次汇总记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/completion_validation.json) 核验当前配置与两段原生归档一致、运行实现一致、九个材料/耦合/抓取实现与此前认可版本字节一致。归档后只有离线路径检查器改为更保守的最深下探假设，它不参与原生执行。[终态指垫补充复核](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/wet_native_contact_01/final_pad_support_review.json) 说明右手该侧在完成提起与额外保持后均仅一个节点满足额外终态门限；第二近节点超过原门限约 0.524 mm，并非舍入误差。四个固定代表保留原材料身份和原手归属；已有保存数据不能定位接触数量减少的具体时刻，也没有测量真实夹持力。

## 2026-11-12 完整仿真

[完整中文视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/continuation_review/cotton_contact_first_full_cn.mp4) 记录本轮从干布初态开始，经过对角夹取、搬运铺布、六轮擦拭与两次真实提布换区，到完全吸液和提起湿布。生产动作在 **87.05 s** 完成，另保持 **4 s**，最终物理时刻 **91.05 s**。这次使用当前“先接触、后闭合”的默认控制器，棉布和吸液参数保持认可版本。

运行被用户中断后，从本轮 **38.7 s** 的 `after_reposition_1.pkl` 恢复。独立审计确认恢复前后的原生布、液体、刚体、吸液账本、抓取及任务状态一致；两段采用同一物理和生产代码。视频只采用 `run_01` 至该检查点的前缀和 `run_02` 后缀，排除中断前已录制但恢复后重新计算的支线，也没有混入历史控制器录像。它是完整状态接续的仿真，**不是一次不中断执行**。[来源索引](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/source_index.json) 与 [视频逐帧来源](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/continuation_review/composition_provenance.json) 保留这一范围。

- **任务结果：16/16 项通过。** 1489 份液体、共 **10.48256 g** 全部进入布内，自由液体为 0；原生湿布质量 **19.55456 g**。独立液体质量重建误差约 **3.47×10⁻¹⁸ kg**，逐顶点湿质量加载误差最大约 **8.47×10⁻²² kg**。额外保持结束时，湿布最低点高于桌面 **11.1873 cm**，吸收液体相对真实主提起前上升中位值 **23.2619 cm**。
- **四次抓取顺序通过。** 共 45,525 个外步无缺漏；每次保持开口下降，真实接触后闭合，双手原抓取门限通过后主提起。最终抓取在 **82.15 s** 确认，**82.35 s** 新鲜接触复查后提起。
- **额外终态接触检查通过。** 在 **87.05 s / 91.05 s** 两个原生检查点，四指垫均分别为 **2、5、7、2** 个接触节点，保持原材料对角身份。上次单独湿抓的终态未通过项没有出现在本轮终态；但第一次换区提起期间，一个指垫在 17 个采样只有 1 节点，因此额外“全程每指至少 2 点”诊断仍为未通过，原每指至少 1 点的持续支持规则未丢失。
- **严格材料检查仍未通过。** 原始总结果 **17/19、`passed=false`**，经/纬检查失败状态未修改。湿布完成提起时经/纬峰值约 **2.7251% / 3.1416%**，高于原 **2.2%** 阶段门限；额外保持结束约 **2.7170% / 3.1200%**。**66.5 s 第五轮擦拭**记录局部最大边伸长 **100.5878%**，同一采样全边 P99 为 **0.2847%**、53 条边超过 5%；不能用主体布面低伸长掩盖该局部瞬态。全轨迹采样 P99 峰值为 **3.6464%**。这些指标说明动作完成不等于全过程材料验收通过。

原始 [运行报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/run_02/robot_wipe_validation.json)、[独立质量与状态审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/run_02/independent_audit_summary.json)、[四次抓取与指垫审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/four_grasps_final_review.json) 均保留，旧版报告未覆盖。4 秒保持有录像、每外步夹爪/TCP、周期性指垫支持及布最低高度采样；未保存全节点连续速度，因此不将它称为全节点振动能量验收。抓取仍为接触门控的理想无滑移约束，尚未验证真实力闭合。

重跑同样的完整记录及额外保持，可使用 [完整记录入口](/home/chloe/Desktop/genesis-local/soy_cloth_scene/run_complete_contact_first.py)，输出目录必须为空：

```bash
cd /home/chloe/Desktop/genesis-local/soy_cloth_scene
PYOPENGL_PLATFORM=egl GS_CACHE_FILE_PATH=/home/chloe/Desktop/genesis-local/.cache/genesis XDG_CACHE_HOME=/home/chloe/Desktop/genesis-local/.cache /home/chloe/Desktop/genesis-local/.venv/bin/python run_complete_contact_first.py --output output/my_complete_run --hold 4
```

不传 `--resume` 即从新初态完整运行；`--resume` 仅接受经过物理签名检查的受支持任务检查点。当前报告记录的物理时间与录像时间均不是实际计算耗时。
