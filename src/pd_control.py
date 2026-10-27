"""原 ArmLab 二连杆模型的固定目标 PD 仿真实验。"""

from pathlib import Path

import mujoco
import numpy as np

from _validation import positive_scalars


def simulate_pd_fixed_target(
    model_path,
    q_target,
    Kp=(2.0, 2.0),
    Kd=(0.2, 0.2),
    simulation_duration=3.0,
    torque_limit=1.0,
):
    """在给定的仿真时间内，使用 PD 控制器将二连杆模型移动到目标角度。"""
    q_target = np.asarray(q_target, dtype=float)
    Kp = np.asarray(Kp, dtype=float)
    Kd = np.asarray(Kd, dtype=float)
    for name, values in (("q_target", q_target), ("Kp", Kp), ("Kd", Kd)):
        if values.shape != (2,) or not np.all(np.isfinite(values)):
            raise ValueError(f"{name} must be a finite array of shape (2,)")
    if np.any(Kp < 0) or np.any(Kd < 0):
        raise ValueError("PD gains must be nonnegative")
    positive_scalars(simulation_duration=simulation_duration, torque_limit=torque_limit)

    model = mujoco.MjModel.from_xml_path(str(Path(model_path)))
    if (model.nq, model.nv, model.nu) != (2, 2, 2):
        raise ValueError("expected the two-joint, two-motor ArmLab model")
    step_count = int(simulation_duration / model.opt.timestep)
    if step_count < 1:
        raise ValueError("simulation_duration must cover at least one timestep")
    # 防止模型再次裁剪指令，使记录的 applied_torques 与实际指令不一致。
    limited = model.actuator_ctrllimited.astype(bool)
    if np.any(model.actuator_ctrlrange[limited, 0] > -torque_limit) or np.any(
        model.actuator_ctrlrange[limited, 1] < torque_limit
    ):
        raise ValueError("torque_limit exceeds the model actuator control range")

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    q_dot_target = np.zeros(2)
    times = []
    positions = []
    errors = []
    raw_torques = []
    applied_torques = []

    for step in range(step_count):
        q = data.qpos.copy()
        q_dot = data.qvel.copy()
        error_position = q_target - q
        error_velocity = q_dot_target - q_dot
        raw_torque = Kp * error_position + Kd * error_velocity
        torque = np.clip(raw_torque, -torque_limit, torque_limit)
        raw_torques.append(raw_torque.copy())
        applied_torques.append(torque.copy())
        times.append(data.time)
        positions.append(q.copy())
        errors.append(error_position.copy())
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)

    return {
        "times": np.array(times),
        "positions": np.array(positions),
        "errors": np.array(errors),
        "raw_torques": np.array(raw_torques),
        "applied_torques": np.array(applied_torques),
        "final_time": float(data.time),
        "final_position": data.qpos.copy(),
        "final_velocity": data.qvel.copy(),
        "final_position_error": q_target - data.qpos,
        "last_torque": torque.copy(),
    }
