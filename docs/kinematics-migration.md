# 运动学代码迁移说明

## 来源与协助范围

用户说明这批代码来自自己的其他代码库；具体仓库路径、版本和原始参考来源待学习者补充。
2026-09-15 按用户要求由助手整理模块、迁移示例与断言，并添加回归检查。
此记录不是学习者个人实验报告；个人预测、推导和理解请按 experiment-template.md 另行记录。

## 模型与接口

遵循 project-scope.md：水平面 2R，默认实验杆长 0.30 m、0.25 m，零位沿 +X，绕 +Z 逆时针为正。
q1 相对 +X，q2 相对第一杆；内部单位为 m、rad、s。

- forward_kinematics 返回 4×4 基座到末端位姿；二维位置取 T[:2, 3]。
- jacobian 返回 2×2 位置雅可比，J @ q_dot 为二维末端速度。
- inverse_kinematics 返回两个 (q1, q2) 几何分支；不可达点抛出 ValueError。
- inverse_kinematics_numerical 返回 (q, success, updates)，不修改输入，不打印。
- max_iterations 表示最大关节更新次数；0 表示仅检查初始位置，最后一次更新后也检查收敛。
- 原重复的二维 forward_kinematics 定义已移除，调用方应从正解模块导入并提取位置。

## 本次改动

保留原 DH、解析逆解、伪逆及 0.1 rad 步长上限；共享正解与雅可比。
使用 np.array(..., copy=True) 兼容项目声明的 NumPy 1.26。
增加有限数值、正杆长、二维输入、正学习率与容差、非负整数更新预算的检查。
示例迁至 examples，原回代与速度断言迁至 tests，补充逐列中心差分与边界回归。
运行方式分别见 examples/README.md 和 tests/README.md。

## 局限与尚未完成项

- 当前运动学为无约束几何计算，不筛选 ±150° 关节限位，也不绕回角度；连接规划器前需处理。
- 奇异初始姿态可能停滞；失败仅表示未收敛，不能证明目标不可达。
- 奇异边界的两个解析分支可能重合；等长两杆折叠到原点时肩角有无穷多解，函数仅给出代表解。
- 三组姿态的个人手算核对、双解和奇异姿态解释仍待确认；阶段未标记完成。

## Verification run (2026-09-15)

- NumPy 2.5.3; pytest: 35 passed. NumPy 1.26 was not separately executed.
- All four documented examples completed successfully.
- Numerical IK: 18 updates, final error 8.431947998e-7 m.
- Original velocity experiment: error 5.310127637e-9 m/s.
