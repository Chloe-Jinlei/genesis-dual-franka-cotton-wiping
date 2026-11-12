# 第三方来源

- **Genesis**：https://github.com/Genesis-Embodied-AI/genesis-world ，版本 1.4.3，固定 commit `ffd119c30118ebb9745fc009349efb2112a8cb76`。引擎工作区在打包时无修改；上游 Apache-2.0 许可见 `genesis-world/LICENSE`。完整 Release 包保留该许可。
- **Franka Panda 资产**：使用固定 Genesis 源树中的 `genesis/assets/xml/franka_emika_panda/`，XML、OBJ、STL 和来源说明一并保留。该目录上游 README 标明 Apache-2.0 及 Franka URDF 来源；其他引擎内资产保留各自原许可。
- **Python 运行依赖**：见 `requirements-lock.txt` 与 `environment.json`，各自遵循上游许可；本仓库不重新授权这些依赖。
- **用户参考资料**：`references/plush_cotton_physics_parameters.pdf` 对应用户提供的《熊与棉布_物理参数设置_plush_cotton.pdf》；`references/gripper_issue_2026-11-11.png` 对应用户指出提前闭合问题的截图。文件内容未修改。

仿真代码、结果与参考资料的上传是本次归档用途；此文件不构成对用户自有材料的额外开源授权。
