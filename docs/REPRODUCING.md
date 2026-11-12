# 安装、重跑与证据范围

此仓库保留本次桌面、双 Franka、棉布、SPH 酱油和吸液耦合的生产代码。`environment.json` 记录产出原始结果的环境，`requirements-lock.txt` 是从当时虚拟环境的包元数据导出的完整版本清单，包含部分非本任务必需的包。Genesis 单独固定到子模块 commit `ffd119c30118ebb9745fc009349efb2112a8cb76`，其源代码工作区在归档时无修改。

打包时没有重新进行全新安装或重跑 GPU 仿真。安装脚本的语法与路径处理进行了检查；这不代表在另一台机器上完成了复现，也不承诺跨 GPU、驱动和编译器的逐位一致结果。这个版本清单未记录所有 wheel 的内容哈希。安装 Genesis 时的隔离构建依赖也不属于已锁定的运行时版本。

打包目录的 **196 项 CPU 回归全部通过**，日志为 [`packaging/cpu-tests.log`](../packaging/cpu-tests.log)。该次检查使用原有已验证虚拟环境，同时让打包目录的 Genesis 和场景代码优先出现在 `PYTHONPATH`；它验证了打包源码及测试输入，并非新环境安装验证，也未启动原生 GPU 仿真。

## 已使用的环境

- Ubuntu 24.04.5，Linux x86_64，Python 3.12.15。
- NVIDIA GeForce RTX 5070 Laptop GPU，约 8 GiB 显存，驱动 595.99.02。
- Torch 2.14.0+cu130、CUDA 13.0、Genesis 1.4.3、Quadrants 1.3.3。
- NumPy 2.5.3、SciPy 1.18.1、MuJoCo 3.15.0、Pillow 11.3.0、Numba 0.68.0。
- FFmpeg 6.1.1；无窗口渲染使用 EGL。

材料和检查点适配器明确要求 Genesis 1.4.3。升级引擎、换成其它 Franka 资源、修改材料配置或更换浮点精度都需要重新验证。

## 系统准备与安装

需要 Python 3.12 及其 venv 支持、适用的 NVIDIA 驱动和可用 EGL/OpenGL；通过仓库克隆安装时还需要 Git。Python C 头文件和 C/C++ 编译工具也应可用，供数值运行时编译本地组件。Ubuntu 对应的常见软件包为 `python3.12-venv`、`python3.12-dev`、`build-essential`、`libegl1`、`libgl1`、`ffmpeg`、`fonts-dejavu-core` 和 `fonts-noto-cjk`。安装脚本不执行 sudo、不安装或修改系统库及驱动；缺少这些组件时先通过系统的软件管理方式准备。

原始脚本使用以下字体路径：

- 运行视频：`/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf`。
- 中文标注和复核视频：`/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc`。

通过 Git 克隆后，在仓库根目录运行：

```bash
git submodule update --init -- genesis-world
./setup.sh
```

完整发布包已经包含同一 Genesis commit 的源文件，无需初始化子模块，解压后直接运行 `./setup.sh`。两种形式均使用 `packaging/genesis-source-manifest.json` 对 1060 个引擎跟踪源文件逐个核验大小及 SHA-256；Git 形式额外检查 commit 和工作区状态。

默认使用 `python3.12`；也可指定解释器，例如 `./setup.sh /path/to/python3.12`。脚本在仓库根创建 `.venv` 和 `.cache`，安装完整版本清单，再以 editable 方式安装固定的 Genesis 源码；Torch 另使用官方 CUDA 13.0 wheel 索引。已存在的 Genesis 源码若版本不符或有本地修改，脚本会停止，不会重置。已存在但损坏的虚拟环境也不会被覆盖。

原环境的 Torch 包元数据版本是 `2.14.0`，实际导入的二进制版本为 `2.14.0+cu130`；版本清单保留元数据值，安装后脚本另检查 CUDA 二进制后缀。尚未通过一次全新安装验证该下载解析过程。

包下载、安装和首次内核编译可能占用较多磁盘及时间。当前 Genesis 的文档、LuisaRender 和 ParticleMesher 子模块未被原始场景使用，安装脚本不会额外递归获取它们。SPH 液体表面重建使用已锁定的 `pysplashsurf`。

## 从新初态运行完整任务

```bash
./run-complete.sh
```

该入口使用原生产记录器，执行完整动作并在结束后额外保持 4 秒；它自动创建带时间戳和随机后缀的独立输出目录，不覆盖归档。无窗口录制和缓存位置由入口设置。首次内核编译后，当前高精度配置在原机器上仍约需每仿真秒数十秒实际计算，整段可能需要一小时以上。

自定输出、额外保持或配置时，参数直接传给原记录器；相对路径从仓库根解释：

```bash
./run-complete.sh --output soy_cloth_scene/output/my_complete_run --hold 4
./run-complete.sh --help
```

显式输出目录必须为空。正常交互入口仍为 `./run-soy-cloth.sh --robot-wipe --hold-open`；静态、CPU 等其它选项参见场景目录中的原始说明。完整记录入口默认使用 CUDA，不提供独立 CPU 开关。

结果包括原生视频、每外步夹爪/TCP 跟踪、指垫支持记录、阶段图片、布形数组、液体账本、完整检查点和验证报告。请依据每次自己的报告判断结果，不要把归档的通过结论套用于新运行。

## 测试与原始证据

不创建原生仿真场景的 CPU 回归可运行：

```bash
.venv/bin/python -m unittest discover -s soy_cloth_scene/tests -v
```

其中路径与模型检查仍会读取固定的 Panda 模型；所需少量历史测试输入已经随 Git 仓库收录，无需为这套 CPU 测试下载全部实验归档。这些测试不能替代实际抓取、液体流动和湿布提起验证。

本次原始完整记录包含从新初态到 38.7 秒的 `run_01` 前缀，及从完全相同原生检查点恢复的 `run_02` 后缀；用户中断前的重复支线未纳入最终视频。生产任务在 87.05 秒结束，额外保持后为 91.05 秒。它不是一次不中断执行。

六轮擦拭吸收了全部 10.48256 克酱油，最终指垫接触检查通过；严格经纬材料检查没有全部通过。局部瞬态边伸长峰值约 100.588%，一次换位阶段的额外每指两节点支持诊断也未通过。原报告保留这些失败，不能用完整视频或任务完成替代材料合格结论。

原始 `output/` 证据按仓库说明从发布附件恢复后，保持其目录结构。完整审计还引用 `output/cotton_fp64_task_review/audit_saved_run.py` 等历史辅助代码，不能仅提取最终视频。原 JSON 和部分审阅工具保留当时的绝对路径；这些记录为了维持哈希而原样归档。异机重新审阅需在单独副本或包装器中进行路径映射，不能直接改写原始证据并仍宣称原哈希有效。`.pkl` 是 Python 原生检查点，只应读取可信的本项目检查点。

材料参考 PDF 的旧绝对路径只是原配置的来源记录，生产代码不读取 PDF；归档副本位于仓库 `references/`。不要为了移动该文档而改写用于原始哈希比对的配置。
