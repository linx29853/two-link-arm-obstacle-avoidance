import numpy as np

def mass_matrix(q):
    q2=q[1]

    m1,m2=0.30,0.25
    a1,a2=0.30,0.25
    lc1,lc2=a1/2,a2/2
    radius=0.01

    I1=(1/12)*m1*(a1**2+3*radius**2)
    I2=(1/12)*m2*(a2**2+3*radius**2)

    M11 = I1 + I2 + m1*lc1**2 + m2*(
        a1**2 + lc2**2 + 2*a1*lc2*np.cos(q2)
    )
    M12 = I2 + m2*(lc2**2 + a1*lc2*np.cos(q2))
    M22 = I2 + m2*lc2**2

    M = np.array([
        [M11, M12],
        [M12, M22],
    ])
    return M

def coriolis_centrifugal(q, q_dot):
    q2 = q[1]
    dq1, dq2 = q_dot

    m2 = 0.25
    a1 = 0.30
    lc2 = 0.125

    # 1. 计算 h
    h=m2*a1*lc2*np.sin(q2)
    # 2. 计算 c1、c2
    c1 = -h * (2*dq1*dq2 + dq2**2)
    c2 = h * dq1**2
    # 3. 返回 np.array([c1, c2])
    return np.array([c1, c2])

def damping_torque(q_dot):
    q_dot = np.asarray(q_dot, dtype=float)
    damping = np.array([0.02, 0.02])

    # 返回电机抵消阻尼所需的力矩
    return damping * q_dot

def inverse_dynamics(q, q_dot, q_ddot):
    M = mass_matrix(q)
    c = coriolis_centrifugal(q, q_dot)
    b = damping_torque(q_dot)

    # 返回三项之和
    return M @ q_ddot + c + b

def computed_torque(
    q, q_dot, q_ref, q_dot_ref, q_ddot_ref, Kp, Kd,
):
    # 1. 位置误差：目标减实际
    # 2. 速度误差：目标减实际
    # 3. 计算 a_cmd
    # 4. 调用 inverse_dynamics(q, q_dot, a_cmd)，返回力矩
    position_error = q_ref - q
    velocity_error = q_dot_ref - q_dot
    a_cmd = q_ddot_ref + Kp * position_error + Kd * velocity_error
    return inverse_dynamics(q, q_dot, a_cmd)

if __name__ == "__main__":
    print(mass_matrix(np.deg2rad([0.0,0.0])))
    print(mass_matrix(np.deg2rad([60.0,0.0])))
    print(mass_matrix(np.deg2rad([0.0, 90.0])))
    print(coriolis_centrifugal(np.deg2rad([0.0, 90.0]), np.array([1.0, 0.0])))
    print(damping_torque(np.array([1.0, -2.0])))
    print(inverse_dynamics(
        np.deg2rad([0.0, 90.0]),
        np.zeros(2),
        np.zeros(2),
    ))
    print(inverse_dynamics(
        np.deg2rad([0.0, 90.0]),
        np.array([1.0, 1.0]),
        np.array([0.0, 1.0]),
    ))
    q = np.deg2rad([0.0, 90.0])
    q_dot = np.array([1.0, 1.0])

    print(computed_torque(
        q, q_dot,
        q.copy(), q_dot.copy(), np.array([0.0, 1.0]),
        np.array([25.0, 25.0]),
        np.array([10.0, 10.0]),
    ))