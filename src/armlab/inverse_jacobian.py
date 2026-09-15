"""使用伪逆和限幅关节步长的局部数值逆运动学"""
import numpy as np

from armlab._validation import positive_scalars
from armlab.forward_kinematics import forward_kinematics
from armlab.jacobian import jacobian


def inverse_kinematics_numerical(
    target, q_init, a1, a2, learning_rate=0.5, tolerance=1e-6,
    max_iterations=200,
):
    """返回目标XY位置（单位：米）对应的 (q, success, updates)"""
    target = np.asarray(target, dtype=np.float64)
    q = np.array(q_init, dtype=np.float64, copy=True)
    for name, value in (("target", target), ("q_init", q)):
        if value.shape != (2,) or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must contain two finite values")
    positive_scalars(a1=a1, a2=a2, learning_rate=learning_rate, tolerance=tolerance)
    if (isinstance(max_iterations, (bool, np.bool_))
            or not isinstance(max_iterations, (int, np.integer))
            or max_iterations < 0):
        raise ValueError("max_iterations must be a nonnegative integer")

    max_step = 0.1
    for updates in range(max_iterations + 1):
        position = forward_kinematics(q[0], q[1], a1, a2)[:2, 3]
        error = target - position
        if np.linalg.norm(error) < tolerance:
            return q, True, updates
        if updates == max_iterations:
            return q, False, updates
        J = jacobian(q[0], q[1], a1, a2)
        delta_q = learning_rate * (np.linalg.pinv(J) @ error)
        step_norm = np.linalg.norm(delta_q)
        if step_norm > max_step:
            delta_q = delta_q / step_norm * max_step
        q = q + delta_q
