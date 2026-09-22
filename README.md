# ArmLab：简单二联杆机械臂规划与控制学习项目

> A learning project for planar robot-arm motion planning and control.
> Status: kinematics code and examples migrated; learning-stage acceptance remains in progress.

本项目采用平面二连杆机械臂，杆长分别为 0.30 m 和 0.25 m。底座位于原点，向右为 +X，向上为 +Y。零位时两杆沿 +X 伸直。q₁ 相对 +X，q₂ 相对第一杆，逆时针为正。代码内部角度使用弧度。



## 最终目标

输入机械臂起终点和静态障碍物，规划全臂无碰撞路径，生成满足速度与加速度限制的轨迹，再通过力矩控制在 MuJoCo 中跟踪并分析误差。

- 规划：直线基线、A*、RRT-Connect。
- 轨迹：路径简化、五次时间缩放。
- 控制：PD、计算力矩控制，以及模型误差和扰动实验。
- 证据：可复现命令、数值验证、指标、失败案例和演示视频。

第一版聚焦二连杆；六轴、视觉、ROS 2、强化学习和真机通信不在当前范围。详见 [项目设计边界](docs/project-scope.md)。

## 目录

```text
src/armlab/   可复用正解、逆解、雅可比与数值逆解
examples/     运动学实验入口与运行说明
tests/        迁移后的验证与回归检查
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

开始可视化时再安装 `.[plot]`，进入动力学阶段再安装 `.[sim]`。运动学运行入口见 [examples/README.md](examples/README.md)，验证命令见 [tests/README.md](tests/README.md)；迁移范围和局限见 [迁移说明](docs/kinematics-migration.md)。

本地可能已有搭建框架时创建的 `.venv`；它不进入 Git，可按上述步骤自行重建。以后发布结果时记录真正运行过的依赖版本。

## 当前进度

- [x] 独立项目框架、学习路线、阶段验收标准。
- [x] 运动学与数值验证。
- [x] 全连杆碰撞检测与构型空间。
- [ ] 规划器与对照实验。
- [ ] 轨迹生成与约束验证。
- [ ] 动力学与控制。
- [ ] 自动评测、演示与发布。

## 学习与来源说明

前置学习记录：[robotics-learning](https://github.com/linx29853/robotics-learning)。参考书：[Modern Robotics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/)。仿真文档：[MuJoCo](https://mujoco.readthedocs.io/en/stable/)。

仓库框架和路线由 AI 辅助整理；现有运动学代码由学习者从自己的其他代码库迁入，助手按请求完成模块整理、示例与测试迁移及回归检查。后续如迁入自己的旧代码或参考第三方实现，记录来源、许可与修改内容；不把阅读、辅助生成或复现直接表述为独立原创成果。

公开发布前检查代码与素材来源并选择许可证；本框架暂未替学习者选择许可证。
