# 实验入口

在项目根目录完成 README 中的可编辑安装后运行：

```powershell
.\.venv\Scripts\python.exe -m examples.forward_kinematics_exp
.\.venv\Scripts\python.exe -m examples.inverse_kinematics_exp
.\.venv\Scripts\python.exe -m examples.jacobian_exp
.\.venv\Scripts\python.exe -m examples.inverse_jacobian_exp
```

| 示例 | 观察点与验证方式 |
| --- | --- |
| forward_kinematics_exp | 保留原有 (0°, 90°) 实验，显示位姿和位置；与自己的手算记录核对 |
| inverse_kinematics_exp | 两个逆解分支、内外边界、不可达点；查看正解回代误差 |
| jacobian_exp | 两关节对末端速度的贡献；比较 J @ q_dot 与前向时间差分 |
| inverse_jacobian_exp | 非奇异初值下的伪逆迭代；查看收敛标志、更新次数及最终位置误差 |

断言已迁到 tests，示例负责展示结果。运行成功不代表已经完成个人学习验收。
当前逆解是无关节限位的几何实验；内边界折叠姿态超出项目 ±150° 限位。
