# ArmLab：机械臂规划与控制学习项目

> A learning project for planar robot-arm motion planning and control.
> Status: repository scaffold only; algorithms and experiments are not implemented yet.

从平面二连杆出发，在没有真机的条件下，逐步实现全连杆避障、轨迹生成、MuJoCo 动力学跟踪和对照实验。

## 从这里开始

1. 阅读 [完整学习路线](docs/roadmap.md)，一次只推进一个阶段。
2. 从第 1 阶段开始，先在纸上明确坐标与单位，再自己写代码。
3. 使用 [实验记录模板](docs/experiment-template.md) 记录预测、结果和修正。
4. 完成验收、能够解释代码后，再提交该阶段的 Git 记录。

## 最终目标

输入机械臂起终点和静态障碍物，规划全臂无碰撞路径，生成满足速度与加速度限制的轨迹，再通过力矩控制在 MuJoCo 中跟踪并分析误差。

- 规划：直线基线、A*、RRT-Connect。
- 轨迹：路径简化、五次时间缩放。
- 控制：PD、计算力矩控制，以及模型误差和扰动实验。
- 证据：可复现命令、数值验证、指标、失败案例和演示视频。

第一版聚焦二连杆；六轴、视觉、ROS 2、强化学习和真机通信不在当前范围。详见 [项目设计边界](docs/project-scope.md)。

## 目录

```text
src/armlab/   以后放可复用模块，目前只有包标记
examples/     以后放各阶段可运行实验
tests/        以后放自己编写的验证
configs/      以后放模型、场景和实验配置
docs/         路线、设计约定、实验记录
assets/       以后放本人实际运行得到的图片和动画
```

## 环境准备

安装 Python 3.11 或更新版本。在本目录中执行以下 Windows PowerShell 命令：

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
```

开始可视化时再安装 `.[plot]`，进入动力学阶段再安装 `.[sim]`。目前没有 CLI 或演示入口；不要把成功安装解释成项目功能已完成。

本地可能已有搭建框架时创建的 `.venv`；它不进入 Git，可按上述步骤自行重建。以后发布结果时记录真正运行过的依赖版本。

## 当前进度

- [x] 独立项目框架、学习路线、阶段验收标准。
- [ ] 运动学与数值验证。
- [ ] 全连杆碰撞检测与构型空间。
- [ ] 规划器与对照实验。
- [ ] 轨迹生成与约束验证。
- [ ] 动力学与控制。
- [ ] 自动评测、演示与发布。

## 学习与来源说明

前置学习记录：[robotics-learning](https://github.com/linx29853/robotics-learning)。参考书：[Modern Robotics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/)。仿真文档：[MuJoCo](https://mujoco.readthedocs.io/en/stable/)。

仓库框架和路线由 AI 辅助整理，核心算法尚未编写。后续如迁入自己的旧代码或参考第三方实现，记录来源、许可与修改内容；不把阅读、辅助生成或复现直接表述为独立原创成果。

公开发布前检查代码与素材来源并选择许可证；本框架暂未替学习者选择许可证。
