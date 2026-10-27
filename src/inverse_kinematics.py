"""解析几何逆运动学，不进行关节限位过滤"""
import numpy as np
from _validation import finite_scalars, positive_scalars


def solve_q2(x, y, a1, a2):
    """以弧度为单位返回正/负肘部分支"""
    finite_scalars(x=x, y=y)
    positive_scalars(a1=a1, a2=a2)
    c2 = (x**2 + y**2 - a1**2 - a2**2) / (2.0 * a1 * a2)
    tolerance = 1e-12
    if c2 < -1.0 - tolerance or c2 > 1.0 + tolerance:
        raise ValueError("目标位置不可达")
    c2 = np.clip(c2, -1.0, 1.0)
    s2 = np.sqrt(1.0 - c2**2)
    return np.arctan2(s2, c2), np.arctan2(-s2, c2)


def solve_q1(x, y, a1, a2, q2):
    """返回所选肘部分支的肩部角度（以弧度为单位）""" 
    finite_scalars(x=x, y=y, q2=q2)
    positive_scalars(a1=a1, a2=a2)
    target_angle = np.arctan2(y, x)
    correction_angle = np.arctan2(a2 * np.sin(q2), a1 + a2 * np.cos(q2))
    return target_angle - correction_angle


def inverse_kinematics(x, y, a1, a2):
    """返回两个 (q1, q2) 分支；如果不可达则抛出不可达异常"""
    q2_positive, q2_negative = solve_q2(x, y, a1, a2)
    return [
        (solve_q1(x, y, a1, a2, q2_positive), q2_positive),
        (solve_q1(x, y, a1, a2, q2_negative), q2_negative),
    ]

