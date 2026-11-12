> 当前版本以用户 PDF 的 180 g/m² `plush_cotton` 参数为基础，双精度、速度重建、形变残差阻尼及干抓反馈见第 11 节；先接触再闭合的修正见第 13 节；当前控制器的完整运行及未通过的材料项见第 14 节。第 1–12 节保留此前材料、控制和验证历史，旧报告未修改。

# 棉织物低伸长：现状诊断与数值验证

本说明前九节记录历史 Genesis 1.4.3 场景中的“布看起来像橡胶”问题。结论是：历史结果确有局部过伸长；优先修正过宽抓取固定区及已经拉长后才被固定的材料边，同时增加拉伸约束迭代。单独降低 `stretch_compliance` 不足以解决问题。当时的弯曲参数在小角抓取基准中已经能够自然下垂，因此该轮没有依据外观大幅修改它；这不是新 PDF 二面角模型的结论。

以下是**未实测标定的低伸长棉织物工程近似**，不是某款棉布的数字孪生，也不是材料认证。

## 1. 棉布假设与文献依据

目标材料假设为不含弹性纤维的机织棉布，当时面密度 0.4 kg/m²。希望在本任务约几十克湿质量及缓慢双臂动作下，经纬向保持较低伸长，同时允许弯曲、皱褶及一定面内剪切。

机织布的伸长、剪切和弯曲需要区分。纱线排列、卷曲和摩擦会使其表现出方向相关、非线性的响应。Wang 等人的测量和建模工作分别处理非线性、各向异性的拉伸与弯曲，说明仅用一个各向同性刚度值不能充分表征不同织物。[原论文](https://graphics.berkeley.edu/papers/Wang-DDE-2011-08/Wang-DDE-2011-08.pdf)

棉织物也并非绝对不可伸长。近期对两类 100% 棉织物的条样实验给出不同经纬向的最大力和对应伸长，其中梳棉样品为 576/615 N、14.7%/19.3%，丝光棉样品为 582/328 N、7.1%/12.1%。这些是高载荷拉伸试验中的结果，不能直接作为擦拭时的正常应变目标；更不能将其对应的破坏或极限伸长设为日常操作允许值。该研究同时指出织物构造、方向及纱线卷曲会改变局部应变。[实验论文与表 2](https://www.mdpi.com/2673-7248/5/4/67)

Goldenthal 等人的低伸长布料研究强调，很多织物在自重下不会产生显著可见伸长，常见模拟为节省计算而容许约 10% 甚至更多伸长，会影响外观。他们通过约束投影，将允许应变作为求解精度目标。这支持本任务采用明确的低应变验收指标；并不提供本块棉布的实测材料常数。[原论文](https://www.cs.columbia.edu/cg/pdfs/131-ESIC.pdf)

## 2. 历史输出的实际应变

本次使用原生、尚未 `build()` 的同配置 PBD 实体重建了参考物理拓扑：1543 个节点、4494 条边、2952 个三角形。参考位置、边和静止长度保存在 `output/material_rest_mesh.npz`。分析的是模拟物理网格，未使用 1107 顶点的显示网格。

边工程应变定义为：

`当前边长 / 原始静止边长 - 1`

正值表示拉伸。整体平移、旋转、垂落或折叠本身不会被记为边长伸长。下表为历史最终帧，不代表整个轨迹的峰值。

| 历史输出 | P95 拉伸应变 | P99 拉伸应变 | 最大拉伸应变 | 超过 5% / 10% 的边数 |
| --- | ---: | ---: | ---: | ---: |
| `robot_wipe` | 1.181% | 10.394% | 50.438% | 61 / 47 |
| `robot_live` | 1.408% | 2.721% | 27.177% | 9 / 4 |
| `robot_final_retry` | 1.429% | 4.461% | 38.187% | 39 / 19 |

最后一组旧动作报告曾通过当时的任务检查，但当时未把布料应变列入验收。因此动作成功与低伸长材料表现是两个独立结论。

额外按平面参考材料 X/Y 方向计算三角形变形梯度，`robot_final_retry` 的 X 向伸长 P99 为 5.978%、最大 45.113%；Y 向为 2.609%、最大 32.681%；绝对剪切角变化 P99 为 6.840°。这进一步表明异常不只是整体下垂。X/Y 在此只是所选材料经纬方向的代理。

完整结果：[历史快照应变报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/material_snapshot_strain.json)。重新计算：

```bash
cd /home/chloe/Desktop/genesis-local/soy_cloth_scene
/home/chloe/Desktop/genesis-local/.venv/bin/python material_validation.py --snapshots
```

参考拓扑必须与被测配置一致；更改网格或重网格化规则后，不应把旧参考文件用于新索引顺序。

## 3. 原生求解器为何不能只调一个 compliance

本机 `genesis/engine/solvers/pbd_solver.py` 的拉伸求解逐轮计算边误差，通过共享节点的 `dpos` 累加位置修正，再统一应用。这是局部 Jacobi 式求解。每条边使用：

- `C = 当前边长 - 静止边长`
- `alpha = stretch_compliance / substep_dt²`
- 修正分母为两个节点的逆质量之和加 `alpha`
- 默认 `stretch_relaxation = 0.3`

当时的历史配置为 4 ms 外层步长、40 个子步，对应 `substep_dt=0.0001 s`。原设置 `stretch_compliance=1e-7 m/N` 使 `alpha=10 kg⁻¹`。干布单节点质量约 1.307e-5 kg，逆质量约 7.65e4 kg⁻¹；两个自由节点的一条边上，`alpha` 只占该分母约 0.0065%。降到 `1e-9` 后该项更小，但本来就接近硬约束，因此预期改变量很小。该轮生产配置见第 7 节，当前 PDF 配置见第 10 节。

原生这段实现没有保存和累积每条拉伸约束的拉格朗日乘子，也没有 XPBD 更新公式中的 `-alpha * lambda` 项。不能据字段名称把它视为完整、迭代次数无关的 XPBD 材料模型。XPBD 原论文通过累计乘子建立顺应性与约束力的一致关系。[XPBD 原论文](https://mmacklin.com/xpbd.pdf)

此外，原生顺序为拉伸、弯曲、粒子碰撞，然后进入刚体接触及抓取约束。后续步骤能够再次改变先前已修正的边长。增加拉伸迭代可以改善收敛，但不能纠正两个都被固定的节点之间已经形成的过大距离，也不能凭空满足相互冲突的抓取几何。

“更小子步”也是可能的收敛路径。Small Steps 的研究比较了相同计算规模下更多子步与更多单步迭代，发现前者可显著改善 XPBD 的约束误差和阻尼。[原论文](https://matthias-research.github.io/pages/publications/smallsteps.pdf) 当时的场景已经使用很小的 0.1 ms 子步，不应直接套用其结论继续增加全局子步；需要把液体计算成本、数值分辨率和本机收敛数据一起比较。

## 4. 独立原生 CUDA 基准

`material_validation.py` 创建独立的布料场景，不包含机器人、桌面或液体，不修改生产场景与吸液扩展。固定区域是两个相邻材料角附近半径 8 mm 的小块，共 8 个节点。先在重力下自由垂落 0.5 s，再用 0.3 s 平滑把两角间距增加 1%，随后保持至 1.2 s。

布面密度设为 0.6 kg/m²，总质量约 30.24 g，作为现有约 20.16 g 干布加约 10.48 g 液体的均匀湿重代理。真实任务中的含液质量是局部的，两者不完全等价。

| 拉伸迭代 | 拉伸 compliance | 弯曲 compliance | 0.5 s 自重 P99 | 最终 P99 | 最终最大应变 | 最低点下垂 | 动态计算耗时* |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 8 | 1e-7 | 1e-5 | 0.605% | 1.188% | 4.516% | 15.84 cm | 1.93 s |
| 32 | 1e-7 | 1e-5 | 0.435% | 1.165% | 4.601% | 16.80 cm | 3.04 s |
| 32 | 1e-7 | 1e-2 | 0.419% | 1.154% | 4.450% | 16.78 cm | 3.07 s |
| 32 | 1e-7 | 1e-1 | 0.436% | 1.158% | 4.086% | 16.77 cm | 2.84 s |

\* 不包含环境加载和场景构建；不是完整双臂任务性能预测。

四种设置在每 0.1 s 采样的历史中均没有超过 5% 的边。32 轮将纯重力状态的 P99 误差降低约 28%，但小幅拉伸后的局部最大值没有同步下降，说明迭代数也不是唯一因素。

布深为 18 cm，默认弯曲参数下最低点已经下垂约 16.8 cm，具备明显的可弯曲性。把弯曲 compliance 从 1e-5 增至 1e-2 或 1e-1，在该基准中未显著增加下垂；不能据这组数据认定后一组“更像真实棉布”。原先过宽的固定材料区会直接限制折叠和垂落，应首先修正。

报告分别为：

- [8 轮基准](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/material_benchmark_8_1e-07.json)
- [32 轮基准](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/material_benchmark_32_1e-07.json)
- [32 轮、弯曲 1e-2](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/material_benchmark_32_1e-07_bend0.01.json)
- [32 轮、弯曲 1e-1](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/material_benchmark_32_1e-07_bend0.1.json)

复现命令（依次运行，不与完整 GPU 任务同时启动）：

```bash
cd /home/chloe/Desktop/genesis-local/soy_cloth_scene
export GS_CACHE_FILE_PATH=/home/chloe/Desktop/genesis-local/.cache/genesis
export XDG_CACHE_HOME=/home/chloe/Desktop/genesis-local/.cache
/home/chloe/Desktop/genesis-local/.venv/bin/python material_validation.py --iterations 8 --backend cuda
/home/chloe/Desktop/genesis-local/.venv/bin/python material_validation.py --iterations 32 --backend cuda
/home/chloe/Desktop/genesis-local/.venv/bin/python material_validation.py --iterations 32 --backend cuda --bending .01
/home/chloe/Desktop/genesis-local/.venv/bin/python material_validation.py --iterations 32 --backend cuda --bending .1
```

## 5. 建议的生产试验与工程限值

当时根据历史 4 ms 配置的基准，建议保持 40 子步、弯曲 compliance 1e-5 和 4 轮弯曲迭代，将拉伸迭代提升至 32。拉伸 compliance 可保持 1e-7，或采用当时生产试验的 1e-9；后者不能被描述为此次低伸长改善的主要依据，也不能等同于实测杨氏模量。该历史建议不是当前 2 ms 配置的子步设置；该轮参数见第 7 节，当前 PDF 参数见第 10 节。

抓取首先约束材料区域：在布面上方预闭合夹爪，下降时避免把大面积布拖成团；只选择真正接触两侧指垫、相互连通的最小材料角块。建立附着前核对候选固定节点之间的原始材料距离。对两个均固定的边或节点对，如果当前几何已经过伸长，应拒绝抓取或重新调整动作，不能靠提高求解迭代掩盖。

本任务建议的工程验收目标为：

- 全过程每个采样时刻报告边应变 P95、P99、最大值与超过 2%/5% 的边数，并同时报告固定—固定、固定—自由、自由—自由三类边。
- 普通自由区域及混合边目标：P99 ≤ 2%，最大拉伸 ≤ 5%。这些是对当前离散分辨率和操作速度的低伸长验收限值。
- 固定—固定边和抓取候选节点对目标：≤ 2%；冲突的固定几何不能交给不可移动节点的求解器修复。
- 同时监测参考材料 X/Y 的伸长及剪切角，避免仅以整体包围盒、下垂高度或重心位移判断拉伸。
- 保留抓取成功、接触证据、质量守恒、液体归零与最终提起检查。低应变不能替代操作成功，操作成功也不能替代低应变。

2% 和 5% 是该轮任务的工程目标，**不是所有棉织物共有的实测正常应变或断裂阈值**。如果这组几何和求解参数仍无法达到目标，应记录失败并调整抓取路径、材料角块、求解顺序或采用更强的约束方法，而不应将检查阈值随结果放宽。

## 6. 尚未实现的真实织物内容

当时的原生模型仍以各向同性边长和二面角约束为主，没有独立的经纬向非线性本构、纱线卷曲恢复、方向相关剪切刚度或实测弯曲曲线。对全部三角形边都施加低伸长约束，也会限制斜向剪切；真实机织布的斜向变形不等同于纱线本身拉长。

若下一阶段要求“真实棉布”材料辨识，应选定具体布样，分别测量经向、纬向、斜向/剪切及弯曲响应，并拟合方向相关的壳或织物模型。这里增加的变形梯度诊断仅用于量化，没有改变原生本构。湿态力学、吸液容量和黏度也仍需独立标定。

本说明和独立基准不会自动修改正在运行的完整双臂任务。生产修复的结论必须来自新一轮完整任务应变历史与操作报告。

## 7. 历史 400 g/m² 验证配置与局部布内输运

本节记录旧模型的最终验证配置与精度问题；上文 4 ms 步长、1543 节点的历史诊断与独立基准不能直接视为当前配置的验证。**最终 4 mm 湿抓预开口方案已通过 18 项任务与指定阶段检查；全过程局部误差仍未全部满足材料目标，具体见第 8 节。** 此前 2 ms/40 子步的 float32 结果存在重力舍入问题，仍不能作为真实重力下的材料验证。

当时 `config_robot_wipe.json` 使用 **2 ms 外层步长、20 个子步、float32**，即每子步 0.1 ms；拉伸 compliance 为 `1e-9`、32 轮，弯曲 compliance 保持 `1e-5`、4 轮。布料采样间距为 **3 mm**、6406 个节点，碰撞壳半径独立设为 **0.75 mm**。空气阻力按参考总面积均分至节点，避免加密网格时总阻力仅因节点增加而放大；壳厚、阻力及织物参数均未实验标定。

双臂从相对布中心 `(-0.13, +0.08)` 与 `(+0.13, -0.08)` 的对角区域抓取；初始材料候选半径 22 mm，初抓空中每指预闭合至 8 mm。每侧指垫仍须至少两个真实接触节点，最终只固定每侧一个不同代表，共两节点/手；所有固定点对的当前距离不得比初始材料距离增加超过 2%。其余接触节点保持自由。初抓及湿抓闭合的指关节 PD 均为 **2000/40**。湿抓预开口独立设为每指 **4 mm**，在 TCP Z=0.775 m 预闭合后下降到 0.770 m；绑定后以 **10000/100** 保持各手实测开度，湿布提起用时 **3.5 s**，再稳定 **1 s**。湿抓几何筛选见第 9 节。释放后的擦拭仍为两手沿 **−Y 方向同步移动 1 cm 再返回**，每段 1.6 s；布料实际位移由接触摩擦决定，不等同于末端目标位移。

`cotton_wicking.py` 增加局部、容量受限的面内输运，生产有效系数 **D=5e-6 m²/s** 是未标定的工程值。模型在参考材料平面中按饱和度差移动已吸收的有限液体份额，保留每份液体的质量与唯一附着三角形/重心坐标：

- 连续漂移每步同时受 **0.2 mm** 和该三角形最小高度的 **20%** 限制。
- 连续漂移无法离开的饱和区域可向一个共边邻面转移；材料位移不超过 **3 mm**，按 `D × dt / d²` 与局部饱和度差累积通量额度，每份液体每帧最多跳一次。路径改变会清除此前额度。
- 接收节点不能超过既有容量，不借用远处干布容量，不跨越材料上不相邻的折叠布层，不删除液体或增加总吸收量。
- 节点质量账目与液体份额账目一致；携带动量混入接收节点，受固定节点约束产生的反作用单独计入动量核对。该算子不解析相对流动惯性，也未保证机械能守恒。

耦合层在下一原生物理步前同步更新节点湿质量、速度、液体附着位置和染深显示；完整状态检查点包含邻面转移目标与累计额度。这个有限份额模型是局部扩散近似，**不是解析纤维孔隙或毛细压力的模型**，不能将系数直接解释为某款棉布的实测吸液性能。

当时的 [CPU 回归日志](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_final_regression_56.log) 记录 **56 项通过**，包含数值分辨率门限、湿抓接触几何、一环应变筛选、材料范围、固定点对伸长、输运容量与质量/动量账目、湿质量接入及状态恢复等检查。这些算子回归不能替代完整原生重力积分、液体与机器人接触运行。

此前 [短原生角抓探针](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_contact_dt002_kp4000/cloth_shape_measurements.json) 在 2 ms 步长、指关节位置增益 4000 下，记录干布提起阶段边伸长 P99 **0.4477%**、最大 **1.723%**；夹取阶段最大伸长 **13.5885%**，21 条边超过 5%。这些记录仍保留，但**其低伸长表现受重力精度错误影响，撤回作为材料验证的意义**，不能据此认定求解参数已经得到真实重力下的低伸长棉布。

上一轮 [cotton_diagonal_wipe 报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_wipe/robot_wipe_validation.json) 已在 **29.7 s** 因自由液体穿桌/非有限状态检查停止，仍有 115 个自由液体粒子，结果为失败。后续 float32 完整候选 [cotton_diagonal_final 报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_final/robot_wipe_validation.json) 在 **23.822 s** 因布料持续上飘而主动中断，吸收量为 **0**，1489 个自由液体粒子未被吸收，同样未通过。

精度诊断发现：`h=0.002/40=5e-5 s` 时，从静止出发的半隐式重力位移约为 `g × h²=2.45e-8 m`；在 `Z≈0.76 m`，float32 相邻数间隔约为 `5.96e-8 m`，半 ULP 为 `2.98e-8 m`。该位移不足以改变存储的位置，而原生 PBD 随后以 `vel=(pos-ipos)/h` 重建速度，使重力增量不能累积。这一机制已由两份独立 CPU 复现支持，并与原生运行中布料上飘、无法正常落布吸液的记录相符。这里的问题来自时间步、坐标尺度和数值精度的组合；增加子步并不必然提高本实现的物理有效性。

`scene.py` 支持 `simulation.precision`。`cotton_contact_fp64` 的 64 位试运行实测约 **70 秒实际时间/1 秒仿真**，已主动取消；它不是完整验证，也不构成 64 位方案已通过的证据。

该轮增加 `numerical_resolution.py`，在场景构建前按最大布料世界坐标 **Z=1.25 m** 检查 `gravity_step=9.81 × h²` 是否严格大于半个 ULP，并在报告中记录 `ratio=gravity_step/ULP`。恢复 `h=1e-4 s` 后，重力位移为 **9.81e-8 m**，该坐标上限的 ULP 约 **1.19209e-7 m**，`ratio≈0.823>0.5`。运行中布料越过配置的高度范围会停止。这个门限仅检查重力增量不会完全被位置舍入吞掉，**不是自由落体精度、接触稳定性、SPH 稳定性或所有舍入误差的证明**；通过后仍需原生动态验证。

当时的 [100 微秒子步短探针](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_contact_substep20/cloth_shape_measurements.json) 恢复了布料下垂，干布提起阶段边伸长 P99 为 **0.874308%**、最大 **4.203191%**，投影面积降至初始值的 **35.76%**。投影变化与下垂现象相符，但不是精确重力标定。同一探针夹取阶段的最大伸长为 **26.7354%**，31 条边超过 5%；因此这只是短诊断，不能宣称全过程满足低伸长验收。

[初次夹取局部接触审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_grip_contact_audit.md) 对上述短探针继续核查：31 条异常边占 18,951 条边约 **0.164%**，涉及 37 个自由节点，均未直接连接四个绑定节点；同一批边在提起快照的最大伸长降至 **1.7900%**。夹取时碰撞壳侵入桌面的最大残差约 **0.6463 mm**，但节点中心仍在几何桌面上方。几何和求解顺序支持“手指底部空间不足、刚体接触投影在拉伸求解后再次扭曲局部网格”的归因，尚未通过修改几何后的对照仿真证实，也未修复。这些实际边长误差必须保留，不能将其解释为真实棉布的均匀弹性伸长，或用提起阶段的低应变替代。

## 8. 历史 400 g/m² 验证结果、来源与分支

[独立终态审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/independent_audit.json) 已通过：最终保存数组中的液体质量、容量、三角面附着和边伸长均经独立复算；按实际手指姿态，两手均有双侧几何接触，18,951 条布边中心线与指垫实体的交叉数为零。此项不证明整个轨迹的边/面连续碰撞或接触力。最终原生湿质量仅与运行报告交叉核对；逐节点原生湿质量和液体分配重建在拾取前完整检查点验证，具体数据限制已写入审计文件。

[cotton_diagonal_small_pinch 最终报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/robot_wipe_validation.json) 为 **18/18 项通过、`passed=true`**。验证前段沿用 `cotton_diagonal_complete` 从初始状态至 **53.85 s** 的运行，在 `before_final_pickup.pkl` 保存完整物理、吸液、输运额度与控制器状态；随后恢复同一检查点，使用修正后的 4 mm 湿抓动作，运行至累计 **61.55 s**。原分支在 56.65 s 的失败末段没有加入成功轨迹，原目录的失败报告也没有改成成功。

最终经历 7 个按压/擦拭周期，**1489 个液体粒子全部吸收、10.48256 g 液体质量误差为 0**。原生湿布质量约 **30.64256 g**，湿布最低点距桌面 **10.8945 cm**，布内液体上升中位距离 **24.2215 cm**。按压后布面投影及近桌面投影均为初始值的 **100.203%**；略高于 100% 是 1 mm 栅格和初始投影基线的测量差异，不是面积增长的证据。

| 最终成功轨迹的提起末帧 | 边伸长 P99 | 最大边伸长 | 超过 5% 的边数 |
| --- | ---: | ---: | ---: |
| 干布 | 0.957681% | 4.879424% | 0 |
| 湿布 | 0.936781% | 4.819909% | 0 |

**18 项通过不表示全过程每条边均满足低伸长目标。** 最终报告的 `maximum_sampled_edge_extension` 为 **37.613263%**，`maximum_sampled_p99_edge_extension` 为 **4.278573%**，保留了初次夹取、铺布等接触阶段的局部误差。干/湿提起末帧的 P99≤2%、峰值≤5% 与第 5 节提出的全过程材料目标是不同范围的结论。未实验标定的各向同性 PBD、节点球接触和理想无滑移锚定仍有局限，不能把任务成功当作完整真实织物模型或连续边面接触已获验证。

[完整验证视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/dual_arm_wipe_verified.mp4) 由前段截至 53.85 s 的录制和成功恢复末段按检查点接续，不是一次未经中断的原始录制。[video_provenance.json](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/video_provenance.json) 记录视频来源，[source_manifest.json](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_small_pinch/source_manifest.json) 记录检查点哈希、代码来源及湿抓参数变更。视频共 962 帧、约 63.498 s，包含前段 815 帧、成功恢复段 117 帧和终态停留 30 帧。两段具有同一物理状态来源，最后 30 帧为已保存终态展示，没有额外物理积分。

[cotton_diagonal_complete 报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_complete/robot_wipe_validation.json) 记录 **56.65 s** 仿真、**7 个**按压/擦拭周期，1489 个液体粒子全部进入布内，共 **10.48256 g**；液体质量误差为 **0**。按压后布面投影及近桌面投影面积均为初始值的 **100.20%**。但该次最终右手夹取未通过真实接触门限，程序停止，未完成湿布提起，报告 `passed=false`。后续试验从最终拾取前的完整物理检查点继续，下表保留原始完整运行的阶段结果。

| 本次完整运行的阶段 | 边伸长 P99 | 最大边伸长 | 超过 5% 的边数 |
| --- | ---: | ---: | ---: |
| 初次夹取 | 0.525339% | 27.531105% | 35 |
| 干布提起 | 0.957681% | 4.879424% | 0 |
| 铺布 | 1.541305% | 21.284465% | 46 |
| 按压/吸液后 | 0.026768% | 0.038351% | 0 |
| 最终夹取失败 | 1.797539% | 51.030011% | 64 |

原失败分支的 18 项检查没有全部通过；上表中的初次夹取和铺布误差属于成功前段也保留的实际测量值，最后一行 51.030011% 则属于检查点之后被舍弃的失败末段，不属于最终成功轨迹。不能把不同分支的峰值混合，也不能因吸液成功或提起阶段表现良好而删除接触阶段误差。

后续三次末段恢复运行均完成动作、全部吸液的携带和湿布离桌，但均未通过湿布提起最大边伸长 ≤5% 的检查。它们各自为 **17/18 项通过、总结果失败**，不能以动作完成代替材料验收：

| 末段恢复运行 | 湿布提起 P99 | 最大边伸长 | 超过 5% 的边数 |
| --- | ---: | ---: | ---: |
| [cotton_diagonal_wet_grip](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_wet_grip/robot_wipe_validation.json) | 1.1279% | 75.9216% | 26 |
| [cotton_diagonal_width_hold](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_width_hold/robot_wipe_validation.json) | 1.8529% | 63.0389% | 39 |
| [cotton_diagonal_wet_preshape，8 mm](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_diagonal_wet_preshape/robot_wipe_validation.json) | 1.4761% | 11.9188% | 14 |

## 9. 历史湿抓几何与边面接触限制

该轮最终方案只改变湿抓动作与绑定前筛选；物理材料仍为第 7 节的 2 ms/20 子步、32 轮伸展，初抓 PD 2000/40 和 8 mm 预开口不变。湿抓启用 `preserve_contact_geometry=True`，当前仍采用 `hand` 链接的理想无滑移锚定；`finger_pads` 选项仅做过 CPU 测试，生产不使用。

中央 pad gap 上限由 **4.5 mm 改为 7.5 mm**。这是明确的数值改变：Panda 中央内面的偏移使其间距比更低的小 pad 间距大 **3 mm**，因而小 pad 最大间距 4.5 mm 对应中央最大间距 7.5 mm。旧中央 4.5 mm 上限实际把小 pad 缝隙限制到最多 1.5 mm。最终方案同时保留双侧各 ≥2 个真实接触节点、初始材料角块身份、TCP≤12 mm 和原局部拓扑路径要求，不能只靠扩大中央 gap 判通过。

绑定前额外要求：候选球壳对两指任一 pad 的侵入不超过 **50 µm**；触发原生自碰撞的选中点对，其球壳重叠不超过同一容差；直接连接两个固定点的真实网格边，其相对参考长度变化为 **±2%**；选中候选节点一环内所有真实网格边的绝对应变均 **≤5%**。在最少固定节点的前提下，优先选择一环最大绝对应变较低的组合，再比较材料范围紧凑性。50 µm 是数值容差，不是绝对无穿透；这些都是建立绑定瞬间的门限，不保证后续不发生卡滞。

湿抓闭合仍为 PD **2000/40**，最终空中预开口缩为每指 **4 mm**。每手通过筛选后，记录实测指关节开度，将后续目标固定为该开度并使用 PD **10000/100** 保持，避免继续闭至零而让指垫扫进 hand-local 固定节点。湿布提起时间为 **3.5 s**，稳定 **1 s**。该组合的任务与指定阶段结果见第 8 节。

[湿布局部冲突审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_wet_grip_conflict_audit.md) 分析的是历史 `width_hold`：39 条超过 5% 的边中，右手 35 条、左手 4 条。右手 35 条热点边有 **27 条中心线穿过 pad 实体**，但端点球壳基本未穿入；最大边 `[945,5799]` 从 **3.215756 mm** 拉长至 **5.242932 mm**，其中 **1.968664 mm** 中心线位于小 pad 内部。原生 PBD 主要处理节点球接触，缺少边/三角面的连续刚体碰撞，因此点的接触检查通过不能排除边穿实体。4 mm 预开口与一环门限改善了本次末段的阶段验收结果，但未补齐这种碰撞求解；不能据 18 项通过宣称所有边面穿透均已消除。

## 10. 历史 PDF 材料移植（float32）：参数、数值差异与验证

本节保留本轮稳定性修改之前的材料移植记录；其中“当前”、配置表和默认湿抓描述均指当时版本。本轮有效设置、独立探针范围与待完成验证见第 11 节。

当前默认材料依据只读的 [《熊与棉布_物理参数设置_plush_cotton.pdf》](/home/chloe/Desktop/熊与棉布_物理参数设置_plush_cotton.pdf) 第 2、4、6 页，以及本地 `reality_latest/runs/plush-cotton-complete-01/source-at-run` 归档中的棉布实现。只移植布料响应，没有引入熊的四面体、体积弹性或形状阻尼。来源、PDF 哈希与本次代码版本见 [source_manifest.json](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/source_manifest.json)，独立复核见 [材料移植审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_material_review.json)。这套参数仍是未实验标定的机织棉布近似。

| 项目 | 当前有效配置与解释 |
| --- | --- |
| 几何与质量 | 长方形 0.28×0.18 m，参考面积 0.0504 m²；面密度 0.18 kg/m²，干质量 9.072 g |
| 物理网格 | 95×61，共 5795 点、11280 面、17074 边；经向 X、纬向 Y 来自初始材料坐标，物理拓扑不经随机重划分 |
| 原生伸展 | compliance 1e-7 m/N，松弛 0.3，每子步 10 轮；仍保留原生边求解，不把它当作完整累计乘子的 XPBD 拉伸本构 |
| 方向伸长投影 | 经/纬/斜向目标 2%/2%/10%，每次原生伸展之后做 24 轮互斥边投影；只限制拉长，不限制压缩 |
| 二面角弯曲 | B=5e-5 N·m，铰链角刚度 `3 B l₀²/(A₁+A₂)`，能量为该刚度乘角误差平方的一半；松弛 0.5、4 轮 |
| 内部阻尼 | 4 s⁻¹ 除以实际最大节点度数 8，逐边速率 0.5 s⁻¹；8 色互斥边上的轴向相对速度耗散，不直接衰减刚体平移或旋转 |
| 空气阻力 | `Fᵢ=-0.5×1.225×1.0×Aᵢ×|vᵢ|vᵢ`；`Aᵢ` 为相邻参考三角形面积之和的三分之一，不是统一 `A/N` |
| 线性阻力 | 1 s⁻¹，对自由棉布节点每子步乘 `exp(-h)`；不作用于 SPH 酱油或刚体 |
| 静/动摩擦 | 0.6 / 0.45；刚体接触沿用场景局部库仑摩擦适配器 |
| 湿质量 | 干节点质量仍由原生面密度计算；吸液非均匀增重写回 native mass，伸展、膜、弯曲、阻尼与空气力继续读取当前质量 |
| 当前默认湿抓控制 | 每指空中预闭合 2 mm、TCP 0.7704 m；每指按 0.1 mm 小步闭合，每步移动 0.05 s、等待 0.05 s，以原严格接触/材料门限反馈停止 |

原熊场景的布料干质量约 **29.39 g** 由其不同几何面积决定，不能作为这块长方形布的质量输入。这里的 **6 g/g** 容量假设得到 **1.08 kg/m²、总容量 54.432 g**；吸收通量 **1 kg/(m²·s)**、局部输运 **D=5e-6 m²/s** 也独立沿用。三者均未实测标定，**PDF 不含这些吸液或毛细输运数据**。质量账目和局部容量检查可以验证实现一致性，不能据此证明真实布样的吸液性能。

数值设置没有逐项照搬参考场景：

- 当前双臂任务为 **2 ms/20 子步、float32**，即 **h=100 µs**；参考归档为 1 ms 子步。沿用较小子步服务于现有 SPH 耦合，同时保留重力位移分辨率检查；该检查不能证明全部积分误差受控。
- **1.5 mm** 碰撞直径与约 3 mm 网格间距分离，保留此前桌面/夹爪的接触设置。它是数值碰撞壳，不能解释为实测布厚，也没有补上连续边/面碰撞。
- PDF 原场景的 **1 s⁻¹ 全场线性阻力** 改为仅作用棉布，避免改变既有酱油动力学。二次空气阻力仍使用原生布料字段，未重复增加另一份空气力。
- 参考弯曲的同步 Jacobi 累加改为 **四节点互斥铰链分组的 Gauss–Seidel 顺序求解**。当前有 16766 个内铰链、17 色，每轮依次更新各色；保留相同 B、能量公式、松弛 0.5 和 4 轮。弯曲乘子在每个 100 µs 子步开始时重置，同一子步各轮继续累积；构建 warmup 也走同一流程。

这项弯曲调度修改有直接数值证据。[CPU 对照](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_bending_diagnostic.json) 在完整参考网格上施加 ±1 µm 棋盘式起伏，仅执行一个子步的四轮弯曲，不含重力、碰撞或机器人。旧同步累加将起伏标准差放大 **4.978 倍**、弯曲能量放大 **25.461 倍**；互斥分组后分别降至初始的 **8.731%** 与 **0.5291%**。该实验隔离了细网格上的调度不稳定，不能解释为真实棉布起皱。单个小扰动的能量下降也不证明所有折叠、接触状态都稳定。

原生静置对照进一步确认了问题与修复：

| 0.5 s 静置试验 | 布节点 Z 范围 | 边伸长 P99 / 最大值 | 范围 |
| --- | --- | --- | --- |
| [参考同步 Jacobi 调度](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_rest_probe/report.json) | 0.760750–0.787125 m | 12.3270% / 17.8810% | 失败的数值调度，不是材料响应依据 |
| [关闭弯曲隔离对照](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_rest_no_bending/report.json) | 全部 0.760750 m | 均约 4.582×10⁻⁸（无量纲） | 诊断设置，不用于正式任务 |
| [17 色互斥弯曲](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_rest_colored/report.json) | 全部 0.760750 m | 均约 4.582×10⁻⁸（无量纲） | 正 B 的正式配置，静置检查通过 |

[本次 CPU 回归日志](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_final_regression.log) 记录 **84 项通过**，包含方向识别、二面角梯度、互斥分组及上述扰动回归、湿质量与阻尼、材料身份和液体账目检查。CPU 与平桌静置都不能替代折叠夹取、搬运、自由液体吸收和湿布提起的完整验证。

后续 float64 投影修复单独通过 7 项 `test_cloth_metrics.py` 回归，记录在干布独立审计；反馈闭合单独通过 [8 项控制测试](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_feedback_regression.log)。这些是各自范围的后续回归，不能相加后声称完整套件已重新跑过 92 项或更多。

**新材料原始完整任务已经结束，结果未通过。** [pdf_cotton_full 报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/robot_wipe_validation.json) 记录从新干态开始累计 **90.9 s**、**13 个**按压/擦拭周期；1489 份液体、**10.482560 g** 全部吸收，自由液体为 0，液体质量误差为 0。原生湿布质量约 **19.554560 g**，最大局部饱和度 **78.6050%**，质量与有限容量检查通过。但最终右手没有满足局部连通材料块内双侧各至少两个接触节点的门限，未完成湿布提起，整体 **`passed=false`、11/19 项通过**。这不是吸液失败，也不能把吸液完成当作整个动作或材料验证成功。第 8 节报告和视频仍只属于旧 400 g/m² 材料版本。

原始干态前段与后续 feedback 湿抓末段的材料测量如下，来源分别为 [原始阶段测量](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/cloth_shape_measurements.json) 和 [feedback 报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/robot_wipe_validation.json)：

| 阶段 | 全边伸长 P99 | 经向最大伸长 | 纬向最大伸长 | 斜向最大伸长 |
| --- | ---: | ---: | ---: | ---: |
| `03_grasped` 初次夹取 | 1.543562% | 13.679572% | 22.460573% | 14.338706% |
| `04_cloth_lifted` 干布提起 | 1.231501% | 2.249339% | 2.217801% | 3.009251% |
| feedback `12_wet_cloth_lifted` 湿布提起 | 2.006165% | 3.845098% | 3.793435% | 5.031833% |

初抓的局部暂态峰值为 **22.460573%**；干布提起末帧中，经、纬各有一条边超过既定 **2%+0.2 个百分点=2.2%** 的阶段验收上限。feedback 湿布提起末帧则分别有 **51 条经向边、13 条纬向边**超出同一上限，斜向全部低于 10.2% 上限。因此**干、湿两个阶段都存在经纬材料残差**，不是仅从前段继承了失败，也没有按结果放宽容差。原始前段继承到续跑的 `maximum_sampled_edge_extension` 为 **37.471517%**，`maximum_sampled_p99_edge_extension` 为 **5.836295%**，不能用提起末帧覆盖全过程峰值。原始 full 报告因缺失湿布提起快照而将斜向联合检查也判为假；feedback 已有湿布提起测量，其斜向检查通过。

后续 [narrow 试验](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_narrow/robot_wipe_validation.json) 从同一新材料的 **86.25 s** `before_final_pickup.pkl` 恢复湿抓，改为每指预开口 **2 mm**、TCP **0.7704 m**；仍于累计 **90.9 s** 因右手真实接触门限失败，11/19 项通过、整体未通过。失败报告保留，不作为成功末段。

再从同一检查点恢复的 [feedback 续跑](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/robot_wipe_validation.json) 增加了每指 **0.1 mm** 的闭合小步，每步 **0.05 s 移动+0.05 s 等待**。原严格门限一旦通过，该手立即保持实测开度，另一手继续检查；绑定时每指保持值约为左手 **1.5018 mm**、右手 **1.4436 mm**，绑定后仍使用 PD 10000/100。门限、材料参数、吸液容量和已吸收状态均不改变。[来源记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/source_manifest.json) 保存检查点哈希及控制变化；它是末段续跑，**不是使用最终控制参数从干态重新完整运行**。

feedback 在累计 **93.5 s** 完成动作序列，湿布最低点高于桌面 **10.3455 cm**，布内液体上升中位距离 **22.8553 cm**；1489 份液体、**10.48256 g** 全部保留，质量误差为 0，原生湿布质量约 **19.55456 g**。最终报告 **17/19 项通过、`passed=false`**，失败项正是干/湿提起经向和纬向伸长联合检查。可以说全部吸液、真实门限湿抓及湿布携带动作已完成，不能说整项材料验收通过。当前默认配置采用反馈闭合、2 mm 预开口和 0.7704 m 湿抓 TCP；原 full/narrow 分支及其来源不覆盖。

结果可查看 [湿抓及提起续跑视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/dual_arm_wipe.mp4)、[湿布末帧全景](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/12_wet_cloth_lifted.png) 与 [近景](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/12_wet_cloth_lifted_detail.png)。该视频记录恢复后的湿抓末段，不把原始 full 的失败湿抓或 narrow 分支拼入完成动作的证据。

另提供约 **93.465 s** 的 [动作回顾拼接](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_review/cotton_wipe_review.mp4)：原 full 第 0–1305 帧（约到 86.2 s）接同材料 86.25 s 检查点的 feedback 末段，并分别添加来源字幕。[provenance.json](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_review/provenance.json) 保存源视频哈希、帧范围和检查点时间。它用于连贯观看前段吸液与后段提起，不能称为新默认从干态完整重跑或一次未经中断的验证，材料结论仍为 17/19 项、整体未通过。

[独立终态审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_pickup_feedback/independent_audit.json) 复算了液体保留、局部容量、布面附着与原始方向门限，并按实际终态手指姿态确认双侧几何接近；17074 条布边中心线与作者定义的指垫盒交叉数为 0，但仍有 3 个自由节点的碰撞壳侵入指垫约 0.09–0.11 mm，不能据中心线结果声称全部接触无穿透。该审计同样为整体未通过，保留经纬超限结论。几何接近不证明力闭合，指垫盒中心线检查也不覆盖全部手指网格、面内部或连续碰撞。最终 `.npz` 不含全部液体面索引/重心权重与逐点 native mass，因此完整逐点质量重建在拾取前检查点验证，终态核对保存的液量/容量和报告原生总质量；不声称已经重载最终全部原生字段。

[dry_material_audit.json](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/dry_material_audit.json) 使用保存的初始与提起快照、原始参考拓扑和 float64 边长计算独立确认了经/纬各一条超限，排除了参考位置或方向分类错误。另有一个独立的投影面积精度问题：本次进程载入的旧指标实现以 float32 处理实时节点坐标，初始覆盖比记录为 98.634168%；现生产 `cloth_metrics.py` 已统一使用 float64 计算投影。旧实现保存在 [source_at_run/cloth_metrics.py](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/pdf_cotton_full/source_at_run/cloth_metrics.py)，其哈希与运行来源记录匹配，原始报告没有覆盖。审计中的离线修正初始面积为 **0.050299 m²、覆盖比 100%**；干布提起面积仍为 **0.022433 m²、覆盖比 44.599296%**。这些带明确来源的离线测量不改变物理快照、方向应变或验收门限，也不是把本次物理积分改为 float64。其余未离线复核的运行面积值应继续按归档算法理解。

经/纬/斜向 **2%/2%/10% 仅是投影目标，不是连续时间硬限**。当前阶段验收按方向检查干/湿提起末帧，并允许 0.2 个百分点的数值残差；后续弯曲、刚体/自碰撞和锚定仍可能改变已投影边长。应保留每阶段及采样历史的方向峰值与超限数量，不能将斜向允许 10% 的新模型套用旧统一 5% 上限，也不能把阶段通过写成全过程处处合格。方向边限制仍未标定经纬非线性本构、独立剪切、湿态模量或纤维级毛细行为，节点接触与理想无滑移抓取的边界继续存在。

可弯曲、下垂和折叠不表示材料必须大幅拉长；本说明的伸长只比较同一物理边的当前长度与初始长度。斜向网格边较宽的投影目标用于近似机织布斜向形变，不能当作纱线在各方向均可伸长 10% 的实验结论。已经记录的接触局部过伸长应继续作为数值误差报告，不能以“柔软”或“像橡胶”替代材料测量。


## 11. 双精度与提起后振动修正：当前设置和验证边界

本节记录稳定性修正完成时的配置与独立探针，彼时默认机器人配置与探针候选一致；[默认配置及验证汇总](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_settling_fix/final_validation.json) 记录默认提升后的源码、配置和 129 项 CPU 回归结果。

本轮不更换 PDF 的基础弹性参数：面密度 0.18 kg/m²、原生 stretch compliance 1e-7 m/N、松弛 0.3、经/纬/斜向目标 2%/2%/10%、B=5e-5 N·m、弯曲松弛 0.5 和 4 轮保持；95×61 规则物理网格、碰撞壳、摩擦、空气力及既有吸液容量也保持。以下变化专门处理积分分辨率、耗散作用范围和夹取时的几何冲突，不能称为新的实测棉布标定。

| 项目 | 本轮设置与作用 |
| --- | --- |
| 原生精度 | `simulation.precision="64"`；刚体/PBD/SPH 使用双精度，dt=0.002 s、20 子步仍保持 |
| 保留积分速度 | `preserve_integrated_velocity=true`，每子步缓存原生力积分后的预测状态，再按 `v_pred+(x_projected-x_pred)/h` 更新速度 |
| 内部耗散 | `internal_damping_model="rigid_motion_residual"`、4 s⁻¹，替换旧逐边轴向模型；拟合自由且活跃节点的质量加权刚体运动，仅衰减残差 |
| 局部弯曲速度耗散 | `bending_damping_ratio=0.5`，按二面角速度作隐式局部阻尼；不是整布临界阻尼比 |
| 其他阻力 | 棉布线性阻力仍为 1 s⁻¹；空气密度 1.225 kg/m³、Cd=1.0、参考节点面积权重不变 |
| 干抓反馈 | `dry_grasp_feedback_closure=true`，空中每指预开口 0.002 m，TCP 0.7704 m；每指 0.1 mm 小步，0.05 s 移动与 0.05 s 等待 |
| 干/湿共同门限 | 同一材料角块、真实双侧各 ≥2 节点、每侧一个不同代表、原拓扑路径；中央 pad 上限 7.5 mm，球壳侵入容差 50 µm，直接固定邻边 ±2%、候选一环边 ±5% |
| 绑定后控制 | 各手通过即保持实测指关节开度，PD 10000/100；仍是 hand 链接上的理想无滑移约束 |
| 吸液精度 | 工作账本为 float64，公共位置/速度 setter 直接按原生 `gs.tc_float` 转换；湿质量写回保持 native 字段 dtype，视觉颜色 float32 不反馈物理 |

旧原生速度重建用两个绝对世界位置之差除以子步时长。在 float32 和约 1 米世界尺度下，小的阻尼/加速度更新可能在位置量化时丢失；预测速度加约束位移的形式在精确算术下等价，却能保留已计算的积分增量。该修正不消除绝对位置投影本身的舍入，也不保证后续刚体接触与附件处理不输入能量。提高原生精度进一步减小这种位置量化；不是增加刚度、放宽应变或直接清零自由布速度。

新的 4 s⁻¹ 算子对自由集合按当前湿质量拟合 `v_COM + omega×(x-x_COM)`，剩余速度乘 `exp(-4h)`。它在固定几何下保留自由集线动量/角动量，固定节点不参与拟合且速度不改；整体刚体摆动和转动故意保留。该思路对应 [Müller 等 PBD §3.5](https://matthias-research.github.io/pages/publications/posBasedDyn.pdf)，本实现增加自由集边界、指数时间率和退化惯量伪逆。它是全布非局部工程近似，不是局部纤维黏弹性，也不与旧逐边轴向 4 s⁻¹ 等效。PDF 的 4 s⁻¹ 数值被沿用，不代表 PDF 已验证这种离散耗散模型。

局部弯曲速度阻尼读取当前湿质量，按铰链角向逆质量 W 和刚度 k 取 `c=2ζsqrt(k/W)`。ζ=0.5 是新增未标定假设，不能用于声称低频整布模态达到临界阻尼。固定点运动的支撑功另行记账。完整 CPU 机制分析见 [mechanism_notes.md](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_settling_fix/mechanism_notes.md) 和 [cpu_mechanism_analysis.json](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_settling_fix/cpu_mechanism_analysis.json)；这些分析针对旧保持轨迹，不是本轮完整任务结果。

此前只有湿抓启用严格 `preserve_contact_geometry`，干抓可能在闭合到零后才检查支持集合。本轮将干抓切到与湿抓同一严格门限和反馈停止策略，并把每指预开口由 8 mm 改为 2 mm、初抓 TCP 改为 0.7704 m。中央 pad 几何上限对干抓由旧 4.5 mm 变为 7.5 mm，同时增加其非侵入及局部压缩筛选；这是有记录的控制和几何检查变化，不能宣称沿用了历史干态前段验证。每一手通过后保持实测开度，避免继续闭合扫入已固定材料，但节点级刚体碰撞仍不能排除边/面穿透。

这一研发阶段的验证记录如下；两个独立探针不得合并成不存在的完整运行。后续五段同材料动作接续另见第 12 节：

| 验证 | 状态与结果 | 证明范围 |
| --- | --- | --- |
| 本轮完整 CPU 回归 | **[CPU 回归日志](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_settling_fix_default_regression.log)：129 项通过，7.216 s** | 算子不变量、接口、控制与精度回归；不是原生流程 |
| 双精度新干态实际抓取、提起和静止保持 | **[干抓保持验收](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_fp64_review/settling_validation.json)：保持 4 s；首秒平均 RMS 3.2889343 mm/s，末秒 0.1496455 mm/s；末秒运动范围 P95 0.10386666 mm，TCP 窗口内最大两两位移 ≤2.34×10⁻¹⁵ m。保持检查通过，但两条纬向边超过 2.2%，材料方向检查失败** | 干抓及保持；不含吸液或湿布动作 |
| 双精度原生短时吸液 | **[短时吸液报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_wet_precision/report.json)：0.5 s、8 项通过；吸收 227 份、1.59808 g，湿布 10.67008 g，原生位置/速度/质量均 float64；液体质量误差 1.73×10⁻¹⁸ kg、湿质量差 −4.34×10⁻¹⁹ kg、附着误差 0；阻尼最后一次读取 10.66304 g** | 约 0.5 s 局部吸液、湿质量和精度通路；不是完整擦拭 |
| 后续同材料干态至湿布提起动作链 | **已按第12节五段原生检查点接续完成；任务通过，严格经纬检查未通过** | 最终控制配置仍未从新干态一次不中断重跑；不复用第10节旧结果 |
| 本机计算代价 | **约 48.29 秒实际时间／秒仿真；本机 NVIDIA GeForce RTX 5070 Laptop GPU，[原生运行日志](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_settle_fp64.log)中 9.5–13.0 s 保持段用时 169.0 s，含录制与控制开销，排除初始化和内核编译** | 仅适用于记录配置；不能搬用历史 70 倍估计 |

稳定性验收窗口严格采用 **3≤t<4 s，共 100 个保存样本**。预先定义的目标是节点运动范围 P95≤1 mm、平均自由节点速度 RMS≤5 mm/s、窗口内 TCP 运动≤1 µm，三项均通过；末秒质心 Z 峰峰值为 **0.657768 µm**。[流程前后对照](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_fp64_review/robot_comparison.json)显示旧保持末秒 P95 为 **21.02428 mm**、新版 **0.10386666 mm**。旧版从历史检查点恢复，新版从干态重新抓取，且绑定节点和多个模型/控制设置改变；这是流程前后对照，不能作同态单变量因果证明。视频为 [新版实际抓取与保持视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_fp64_review/new_robot_full.mp4) 与 [保持阶段前后对照视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_fp64_review/robot_hold_before_after.mp4)，均不覆盖酱油擦拭和湿布提起。

方向应变结果单独保留：`04_cloth_lifted` 的纬向最大值为 **2.278154%**，`after_stationary_hold` 为 **2.267258%**；两个阶段均有 **2 条纬向边超过 2.2%**。保持末帧经向为 **2.116557%**、斜向 **2.016060%**，各自通过。因而保持检查通过，但材料方向及“保持+材料”联合检查均失败；稳定性改善没有使这两条边转为合格，也没有更改 0.2 个百分点数值残差。

短时吸液结束仍有 **1262 份自由液体**。阻尼诊断位于最后一个原生子步，先于最后一次外层吸收，因此其读取的 **10.66304 g** 比外层结束湿质量 **10.67008 g** 少一个 **0.00704 g** 液体份额；这是明确的算子先后顺序，不是湿质量丢失。原生逐节点质量写回、液体/布内账本与附着插值通过该探针的检查，不能推导出完整折叠擦拭和湿布携带已验证。

短时吸液探针只在初始化时把布放到配置液滩上方，然后执行原生重力和普通耦合；没有构建后的粒子摆位，不驱动机械臂完成任务。应验证原生位置/速度/质量 dtype、有限局部容量、液体账本、吸液增重、阻尼读取当前湿质量，以及附着液体位置与材料重心插值一致；只检查输出 dtype 不足以发现中途 float32 量化。当前修复删除六处耦合 setter 前的 float32 强制转换，历史干抓进程载入的旧源码另有 [运行来源归档](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_robot_settle_fp64/extra_source_manifest.json)，不可与随后吸液探针的源码哈希混为一份。

float64 和新增耗散/控制改变了运行模型。旧 float32 检查点不应跨精度恢复；模型签名及原生数组 dtype 检查须通过，新版采用自己的干态来源。第 10 节 17/19 仍是其原分支的失败结论，第 8 节 18/18 也仍只属于更早材料。在本节记录时点，仅干抓保持与短时吸液分别通过对应检查，尚无完整动作证据；随后第 12 节提供了五段原生接续的完整动作与吸液携带结果，仍保留干/湿方向材料失败。干/湿提起经纬 2% 与斜向 10% 的投影目标、另列的 0.2 个百分点数值残差和各阶段峰值记录不随稳定性修正放宽。


## 12. 用户认可材料后的完整动作接续、终态审计与材料残差

用户已认可第 11 节的布料表现，本轮继续完成机器人搬运、铺布、擦拭和湿布提起，**未改变该材料的物理设置**。[最终原生报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_tail_wipe/run/robot_wipe_validation.json) 在累计 97.95 s / 8 个完整周期结束，`failure=null`、`task_passed=true`；16 项动作、接触、质量、容量、覆盖、碰撞、跟踪及绑定长度要求全部通过。原始报告仍为 **17/19、`passed=false`、`strict_material_passed=false`**，经/纬方向检查未通过。接受当前布料外观不等于删去这些方向门限或实验标定。

### 12.1 完成的控制变化与真实状态来源

解除抓取后恢复双手正常夹指 PD 2000/40，防止绑定保持的 10000/100 增益遗留到松开/按压。起始擦动为世界 −Y3cm，出程和回程各 1.6 s，出程额外停留 0.75 s。完整第 2 轮之后真实夹住原材料对角、抬至 TCP 0.96 m，将布中心移到酱油中心加 `[-0.06,-0.045]` m，铺下释放，下一轮 +Y1cm；第 3 轮后再真实提布至酱油中心加 `[-0.06,0]` m，随后 +Y3cm。第 7 轮后仍有两份液体，因此从完整周期检查点加入显式 +Y4.5cm 行程计划，第 8 轮完成全部吸液。所有选区均来自保存状态的离线诊断，不是相机感知或在线自主策略。

从第 3 轮后的湿抓开始，控制器以原参考材料点和实际平布 XY 做无缩放刚体拟合；空手先升至安全高度 0.80 m，再共同对齐观测中心/yaw，原材料角部身份和双 TCP 跨度保持。抓、抬、移、铺及后续按压持续使用该布局；世界换区偏移与擦动方向不随 yaw 隐式旋转。近桌面/平展/拟合误差 guard 只检验观测是否有效，不能替代真实 pad 接触门限。最终湿抓重新测得 yaw +9.601797°、中心 `[-0.056153918,0.043107678]` m，随后完成 3.5 s 提起和 1 s 稳定。

| 本轮采用的原生来源 | 已验证的接续边界 |
| --- | --- |
| `output/cotton_fp64_task` | 新干态至干抓提起检查点，0→9.45 s |
| `output/cotton_fp64_wide_wipe/run` | 搬运、铺布及前两轮，9.45→27.90 s |
| `output/cotton_fp64_reposition/retry_run` | 第一次真实换区及第3轮，27.90→44.85 s |
| `output/cotton_fp64_layout_tracking/run` | 实际布局对齐、第二换区及第4–7轮，44.85→82.45 s |
| `output/cotton_fp64_tail_wipe/run` | +Y4.5cm第8轮、最终湿抓与携带，82.45→97.95 s |

这是五段相同物理设置的原生状态接续，**不是采用最终控制配置从新干态一次不中断运行**。各段 checkpoint 哈希、事件和 CSV 前缀及物理配置一致性已独立核对。layout 诊断停止到 83.04 s 的部分第 8 轮、此前异常终止和湿抓失败的尝试均未混入成功链。旧失败文件没有覆盖。可查看 [末段来源记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_tail_wipe/run/source_manifest.json) 及 [独立审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_tail_wipe/run/independent_audit.json)。[完整动作回顾视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/cotton_wipe_complete.mp4) 已完成并核验，共 1484 个原生帧、97.9538 s。五段接续无重帧/漏帧、无变速或停帧，见 [composition_provenance.json](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/composition_provenance.json) 和 [source_frame_map.csv](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/source_frame_map.csv)。观看拼接不改变上述证据范围。最终 1 s 保持的 16 帧（96.954–97.944 s）未见明显大幅反复弹跳，仍有细微自由角变化；21 个同期 CSV 样本最低布高峰峰值 1.182 mm，只是该短窗口观察，不是重新执行 4 s 保持验收。

完整检查点保存 native rigid、PBD/SPH/coupler 数组、湿质量、液体分配账本、抓取归属和累计控制状态。轮数仅计完整周期；中心/yaw、世界擦动行程、已执行换区/行程计划 ID 与历史一起保存。`after_wipe_stroke_1.pkl` 在选择新行程后、下一轮物理动作前保存；恢复不重复执行同一计划，改写或删除已经执行的行程计划会拒绝。默认无计划时不增加物理步骤。最终 `after_wet_lift.pkl` 是 `audit_only_complete` 审计终态，普通恢复入口拒绝继续；保存成功由真实文件与返回状态核对。

### 12.2 原生结果与独立质量、接触审计

| 项目 | 本轮原生结果 |
| --- | ---: |
| 初始 / 最终自由液体份数 | 1489 / 0 |
| 全部吸收质量 | 10.48256 g |
| 干布 / 原生湿布质量 | 9.072 g / 19.55456 g |
| 湿布最低点 Z / 高于桌面 | 0.871018658 m / 0.111018658 m |
| 液体随布上升中位距离 | 0.232704574 m |
| 采样液体总质量最大误差 | 3.46944695×10⁻¹⁸ kg |
| 最大末端跟踪误差 / 原生刚体碰撞样本 | 0.430535 mm / 0 |
| 最终最大局部饱和度 | 0.820164561 |
| 初次铺布投影和近桌投影比 | 均为 99.064116%，按该次原报告栅格测量 |

[独立终态审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_tail_wipe/run/independent_audit.json) 保留 16 项原任务检查，并从快照/完整终态检查点/实际手指姿态重新检查 45 项证据，全部通过且无未知项。最终 native 逐点质量与液体 parcel 的三角面/重心账本均直接导出，重建逐点湿质量最大误差 **8.4703×10⁻²² kg**；液体表面插值位置最大差约 **2.58×10⁻¹⁶ m**。终态保存了真实提起前每份液体的位置，独立重算的上升中位值与原报告完全一致。

最终左手两指各有 2/2 个节点、右手 3/2 个节点满足原 1.05 mm pad 接近门限，选中代表各保持一侧支持和原材料角部身份。17074 条布边中心线未穿过作者定义的 pad 盒。但两个自由节点附近仍有约 **0.105–0.110 mm** 的球壳/pad 局部侵入；通过保存数据检查不代表所有接触误差消失，也不覆盖三角面内部、整个非 pad 手指实体、连续碰撞或真实夹持力。抓取负载尚未回传机器人，仍是 hand 链接上的理想无滑移近似。

### 12.3 严格方向结果和全过程采样残差继续保留

方向验收仍使用经/纬/斜向 **2%/2%/10%**，干/湿提起末帧另允许 **0.2 个百分点**数值残差。两项经/纬联合检查失败，斜向通过；原始检查和数值都没有为任务完成改写。

| 提起末帧 | 经向最大值 / 超2.2%边数 | 纬向最大值 / 超2.2%边数 | 斜向最大值 / 超10.2%边数 | 全边P99 |
| --- | ---: | ---: | ---: | ---: |
| 干布 | 2.063263% / 0 | 2.221524% / 2 | 1.853806% / 0 | 1.099707% |
| 湿布 | 2.518809% / 14 | 3.063012% / 7 | 3.658817% / 0 | 1.782723% |

其他阶段也不是连续时间硬约束的证明：

| 阶段快照 | 最大边伸长 | 全边P99 |
| --- | ---: | ---: |
| 初次夹取 | 5.827114% | 0.147003% |
| 初次铺在液滩上 | 7.525228% | 1.965000% |
| 第一次换区铺下 | 12.338906% | 0.721711% |
| 第二次换区铺下 | 11.197673% | 0.533041% |
| 全部吸液、最终抓取前 | 0.035552% | 0.025721% |
| 最终湿布提起 | 3.658817% | 1.782723% |

整个接续轨迹每 **50 ms** 保存一次指标；采样最大局部边伸长为 **55.485411%**，各采样时刻 P99 的最大值为 **4.010139%**。这是采样而非连续时间上界，不能因为最终布形或P99较小就隐藏局部峰值。有限轮数材料投影后仍有弯曲、刚体/自碰撞和抓取约束，夹点附近会有不兼容接触残差。已有证据不能把所有峰值唯一归因于某一个算子，也不能解释为真实棉布允许这样的橡胶式伸长。

`task_passed` 只把三个明确的干/湿方向峰检查分类为材料诊断；接触、质量、容量、覆盖、碰撞、跟踪和固定材料点对伸长仍须通过。缺失或非法检查、运行错误或未知失败会阻断任务。`strict_material_passed` 不表示全过程每条边处处合格，更不表示纱线非线性、剪切、湿态模量、毛细作用或力闭合已经实测标定。本轮是已认可工程布料下的动作完成，严格材料残差继续公开。

### 12.4 未改材料证据与最终 CPU 回归

基准 `output/cotton_fp64_task/config_used.json` 与最终 `output/cotton_fp64_tail_wipe/run/config_used.json` 只有顶层 `robot_task` 不同；cloth、simulation、coupling、soy、table、robots 均完全一致。15 个材料/物理/抓取实现文件的归档字节也一致，两侧 manifest 均核验过。

| 归档实现 | 基准与最终相同 SHA-256 |
| --- | --- |
| `cotton_material.py` | `3bfcef543382b953fcd1970a9f4e1187bf8d0aadaaf2cce6b2667554e9f22649` |
| `cotton_membrane.py` | `1b1b9db0d1e981f65d7a1e5e808339f43113e5ea7f2ba5790a326144c2a1df87` |
| `cotton_bending.py` | `4e46bfbaa75fd63d9c0a8e0c305f6a74bba2cbf9e93e3431eea6a059a4dd14cc` |
| `cotton_deformation_damping.py` | `2cf9cc80713289c715f0fe1739a00dcff31c6b2a08caa05b5de8807abd70bc9d` |
| `cloth_liquid_coupling.py` | `faaed03ef8b8b9181cec0763b8edab66532e1525d68961d4037551394cea0a05` |
| `robot_grasp.py` | `846a7b61b041be1caf6a5536384f0fafa038eb631736f5e07e5a1ee1e5143afa` |

完整配置段与归档源码的来源映射和一致性检查见上述最终独立审计及其 `source_chain`、`source_archives` 字段。当时默认配置曾提升为成功候选，SHA-256 `c94b8b453a9402a8b9b39dab65347e2b80216c8ddab14936fcd5dea79964cce3`，见 [default_promotion.json](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/default_promotion.json)；此为后续抓取顺序修正之前的配置。

成功候选控制曾通过 [176 项回归](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_tail_wipe/regression.log)。默认提升后，旧基线测试隐式读取新增可选动作，导致 [3 项失败、8 项错误的原日志](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/default_regression.log)。修复仅在测试夹具显式关闭与其基线无关的计划/停留，并新增实际提升配置的组合检查，生产实现、默认配置和门限未改变。[最终完整回归](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/cotton_fp64_full_review/default_regression_after_fixture_fix.log) **177/177 通过，8.000 s**，包括材料算子、湿质量、控制/接续、计划一次执行、布局连续和结束分类；CPU 通过不能替代上述原生结果或材料残差。

## 13. 接触后才闭合的动作修正，材料保持不变

2026-11-11 针对夹爪在空中提前闭合的反馈，默认改为每指 10 mm 张开接近，实际原生完整手指几何接触原布角后才反馈闭合。所有材料、耗散、碰撞、液体与吸液参数保持不变，原绑定门限和两个材料对角身份也不变。[配置差异](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/config_change_review.json) 记录四处动作配置修改；实际运行代码和配置分别随干/湿原生运行归档。

本次只重跑新干布首抓及真实已吸液检查点之后的最终湿抓，没有重跑中间的铺布、按压和擦拭。两段均完成接触后闭合、原双侧门限确认和主提起，各五项顺序检查通过；[CPU 回归](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/final_cpu_tests.log) 为 196/196。不能将这些结果升级为新控制器整轮材料验收。

| 本次提起阶段快照 | 经向最大伸长 | 纬向最大伸长 | 斜向最大伸长 |
| --- | ---: | ---: | ---: |
| 干布完成提起，9.25 s | 2.049897% | 2.192042% | 3.209213% |
| 干布额外保持后，9.75 s | 2.050258% | 2.189201% | 3.216991% |
| 湿布完成提起，98.45 s | 2.452742% | 2.336808% | 3.828571% |
| 湿布额外保持后，98.95 s | 2.450331% | 2.335726% | 约 3.8265% |

干布夹取快照仍有 11.961967% 局部边伸长，整段干抓采样峰值为 13.523086%；P99 不能掩盖这些局部残差。原始方向目标 2%/2%/10% 与阶段额外 0.2 个百分点均未更改。两段额外半秒保持的最低布高峰峰值约 1.060/0.563 mm，视频未见明显大幅反复弹跳，但不替代四秒保持或全部节点运动验证。

[独立湿抓审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_grasp_review/wet_native_contact_01/independent_audit_summary.json) 复核全部 1489 份液体保留、液体 10.48256 g、原生湿布 19.55456 g，质量账目误差约 −3.47×10⁻¹⁸ kg。完成提起检查点逐点湿质量重建最大误差约 8.47×10⁻²² kg；额外半秒后的 final NPZ 不包含完整 native 逐点质量和分配账本，不能与早半秒的检查点要求逐位相同。终态布最低点 Z=0.869217056 m，液体相对真实主提起前位置的上升中位值约 0.233655465 m。

该独立审计仍为未全通过：右手一个指垫终态只有 1 个节点处于原接近门限内，各指固定代表仍有支持，但未满足额外终态复核的每指至少 2 个节点要求。湿布经/纬方向也仍超 2.2% 阶段上限。抓前原门限通过不代表提起全过程所有局部接触和材料误差消失。当前修正的结论仅是提前闭合顺序已改正并原生验证，完整记录和新视频见 [README 本次修正记录](/home/chloe/Desktop/genesis-local/soy_cloth_scene/README_CN.md)。

## 14. 当前控制器完整运行：六轮擦拭、最终湿抓与四秒保持

2026-11-12 完成本轮从新初态开始的完整物理状态链。用户中断后，在 38.7 s 原状态检查点继续；两段生产代码、配置和物理签名一致，恢复边界原生数组独立比较一致。它不是一次不中断执行，也不是混用旧控制器结果。87.05 s 完成全部动作，额外保持至 91.05 s。详见 [本轮运行说明与完整视频](/home/chloe/Desktop/genesis-local/soy_cloth_scene/README_CN.md) 末节。

六轮擦拭吸收全部 1489 份、10.48256 g 酱油；原生湿布质量 19.55456 g，16 项任务要求通过。四次抓取均先接触后闭合，确认后主提起；87.05 s 与 91.05 s 的终态四指垫支持均为 2/5/7/2 节点。第一次换区仍有一个指垫在部分采样只有 1 节点，原持续支持要求通过，额外全程每指 2 节点要求未通过。

| 保存状态 | 经向峰值 | 纬向峰值 | 斜向峰值 | 全边 P99 | 经/纬超 2.2% 的边数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 干布提起 9.25 s | 2.04954% | 2.19423% | 3.19571% | 0.91641% | 0 / 0 |
| 湿布提起 87.05 s | 2.72509% | 3.14162% | 3.94102% | 1.62011% | 10 / 4 |
| 额外保持结束 91.05 s | 2.71703% | 3.12001% | 3.91812% | 1.61719% | 10 / 3 |

原经/纬/斜目标仍为 2%/2%/10%，阶段额外数值残差仍为 0.2 个百分点。**严格经、纬检查未通过，原总报告仍为 17/19、`passed=false`**，用户认可外观没有改变判据。

全过程还存在明显的局部瞬态伸长：66.5 s 第五轮向外擦拭的采样最大边伸长为 **100.587766%**，该时刻经/纬/斜峰值约 **60.6562% / 100.5878% / 70.8036%**；全边 P99 为 **0.284693%**，53 条边超过 5%。这些局部错误不能以主体布面稳定或末态低伸长取代。全轨迹采样 P99 峰值 **3.646375%** 出现在 13.95 s 铺布阶段。原始采样间隔 0.05 s，不保证捕获更短瞬态的绝对峰值；没有保存 66.5 s 的完整节点检查点，无法凭现有数组确定具体最大边及接触因果。

4 秒额外保持有原生录像、夹爪/TCP、指垫支持及最低布高采样，但没有全节点连续速度/动能历史，不宣称已通过全节点振动能量判据。材料和吸液参数本轮没有修改。参见 [原始完整报告](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/run_02/robot_wipe_validation.json)、[独立审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/run_02/independent_audit.json) 与 [完整抓取审计](/home/chloe/Desktop/genesis-local/soy_cloth_scene/output/contact_first_full/four_grasps_final_review.json)。
