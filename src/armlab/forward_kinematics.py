import numpy as np
from armlab._validation import finite_scalars, positive_scalars


def dh_transform(theta, d, a, alpha):
    finite_scalars(theta=theta, d=d, a=a, alpha=alpha)
    '''有效检查'''
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])


def forward_kinematics(q1, q2, a1, a2):
    '''正数检查'''
    positive_scalars(a1=a1, a2=a2)
    T1 = dh_transform(q1, 0, a1, 0)
    T2 = dh_transform(q2, 0, a2, 0)
    return T1 @ T2
