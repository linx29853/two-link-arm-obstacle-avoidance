
import mujoco
import numpy as np

from armlab._validation import positive_scalars
from armlab.trajectory import quintic_trajectory


def simulate_pd_tracking(
    model_path, q_start, q_goal, velocity_limit=1.0,
    acceleration_limit=2.0, Kp=(2.0, 2.0), Kd=(0.2, 0.2),
    torque_limit=1.0, settling_time=1.0,
):
    #返回同一时刻的参考、实际状态、误差与力矩，单位为 SI。
    times, _, _, _ = quintic_trajectory(
        q_start, q_goal, velocity_limit, acceleration_limit, sample_count=2,
    )
    duration = float(times[-1])
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    Kp, Kd = np.asarray(Kp, dtype=float), np.asarray(Kd, dtype=float)
    for gain in (Kp, Kd):
        if gain.shape != (2,) or not np.all(np.isfinite(gain)) or np.any(gain < 0):
            raise ValueError("PD gains must be finite nonnegative arrays of shape (2,)")
    positive_scalars(torque_limit=torque_limit, settling_time=settling_time)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    if (model.nq, model.nv, model.nu) != (2, 2, 2):
        raise ValueError("expected the two-joint ArmLab model")
    limited = model.actuator_ctrllimited.astype(bool)
    if np.any(model.actuator_ctrlrange[limited, 0] > -torque_limit) or np.any(
        model.actuator_ctrlrange[limited, 1] < torque_limit
    ):
        raise ValueError("torque_limit exceeds the model control range")
    data = mujoco.MjData(model)
    data.qpos[:] = q_start
    mujoco.mj_forward(model, data)
    delta = q_goal - q_start

    def reference(t):
        if duration == 0 or t >= duration:
            return q_goal.copy(), np.zeros(2), np.zeros(2)
        u = np.clip(t / duration, 0.0, 1.0)
        s = 10*u**3 - 15*u**4 + 6*u**5
        ds = (30*u**2 - 60*u**3 + 30*u**4) / duration
        dds = (60*u - 180*u**2 + 120*u**3) / duration**2
        return q_start + s*delta, ds*delta, dds*delta

    records = {key: [] for key in (
        "times", "positions", "velocities", "reference_positions",
        "reference_velocities", "reference_accelerations", "errors",
        "raw_torques", "applied_torques",
    )}
    step_count = int(np.ceil((duration + settling_time) / model.opt.timestep))
    for _ in range(step_count):
        q_ref, v_ref, a_ref = reference(data.time)
        error = q_ref - data.qpos
        raw = Kp * error + Kd * (v_ref - data.qvel)
        torque = np.clip(raw, -torque_limit, torque_limit)
        values = (data.time, data.qpos, data.qvel, q_ref, v_ref, a_ref,
                  error, raw, torque)
        for key, value in zip(records, values):
            records[key].append(np.array(value, copy=True))
        data.ctrl[:] = torque
        mujoco.mj_step(model, data)
    result = {key: np.asarray(value) for key, value in records.items()}
    result.update(
        trajectory_duration=duration, final_time=float(data.time),
        final_position=data.qpos.copy(), final_velocity=data.qvel.copy(),
        final_position_error=q_goal - data.qpos,
    )
    return result
