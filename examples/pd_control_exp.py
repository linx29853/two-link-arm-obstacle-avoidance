"""固定目标 PD 实验入口：设置参数、调用仿真并展示结果。"""

from pathlib import Path

import numpy as np

from armlab.pd_control import simulate_pd_fixed_target


def main():
    model_path = Path(__file__).resolve().parents[1] / "configs" / "two_link_arm.xml"
    q_target = np.array([-0.2, -0.4], dtype=float)
    Kp = np.array([2.0, 2.0])
    Kd = np.array([0.2, 0.2])

    result = simulate_pd_fixed_target(
        model_path,
        q_target,
        Kp=Kp,
        Kd=Kd,
        simulation_duration=3.0,
        torque_limit=1.0,
    )

    print("仿真时间（s）：", result["final_time"])
    print("目标角度（rad）：", q_target)
    print("最终角度（rad）：", result["final_position"])
    print("最终速度（rad/s）：", result["final_velocity"])
    print("最终位置误差（rad）：", result["final_position_error"])
    print("最后一次控制力矩（N·m）：", result["last_torque"])
    print(
        "记录数量：",
        len(result["times"]), len(result["positions"]), len(result["errors"]),
        len(result["raw_torques"]), len(result["applied_torques"]),
    )
    print("记录起止时间（s）：", result["times"][0], result["times"][-1])


if __name__ == "__main__":
    main()
