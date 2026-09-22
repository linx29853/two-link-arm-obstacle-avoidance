from armlab.simple_check import pengzhuang
import numpy as np
from armlab.forward_kinematics import forward_kinematics

def is_pose_valid(q1, q2, a1, a2, xc, yc, r, rod_radius):
    limit=np.deg2rad(150)
    if q1 < -limit or q1 > limit or q2 < -limit or q2 > limit:
        return False

    O = np.array([0.0, 0.0])
    E = np.array([
        a1 * np.cos(q1),
        a1 * np.sin(q1),
    ])
    T = forward_kinematics(q1, q2, a1, a2)
    tip = T[:2, 3]
    collision_1 = pengzhuang(
        O[0], O[1],
        E[0], E[1],
        xc, yc, r, rod_radius,
    )
    collision_2 = pengzhuang(
        E[0], E[1],
        tip[0], tip[1],
        xc, yc, r, rod_radius,
    )
    collision = collision_1 or collision_2
    return not collision

if __name__ == "__main__":
    # 名称、关节角（度）、圆心（m）、预期有效性
    cases = [
        ("两杆安全",       (0, 90),   (0.10, 0.40), True),
        ("第一杆碰撞",     (30, 45),  (0.15, 0.08), False),
        ("第二杆碰撞",     (0, 90),   (0.30, 0.125), False),
        ("q1 超过上限",    (151, 0),  (1.0, 1.0), False),
        ("q1 超过下限",    (-151, 0), (1.0, 1.0), False),
        ("q2 超过上限",    (0, 151),  (1.0, 1.0), False),
        ("q2 超过下限",    (0, -151), (1.0, 1.0), False),
        ("限位边界允许",   (150, -150), (1.0, 1.0), True),
        ("另一组限位边界", (-150, 150), (1.0, 1.0), True),
    ]

    for name, angles_deg, center, expected in cases:
        q1, q2 = np.deg2rad(angles_deg)
        xc, yc = center

        actual = is_pose_valid(
            q1, q2,
            a1=0.30,
            a2=0.25,
            xc=xc,
            yc=yc,
            r=0.04,
            rod_radius=0.01,
        )

        assert isinstance(actual, (bool, np.bool_)), (
            f"{name}：应返回布尔值，实际返回 {actual!r}"
        )
        assert actual == expected, (
            f"{name}：预期 {expected}，实际 {actual}"
        )
        print(f"通过：{name} → {actual}")

    print(f"\n全部 {len(cases)} 个案例通过")
