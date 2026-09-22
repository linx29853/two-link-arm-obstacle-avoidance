import numpy as np
from armlab.forward_kinematics import forward_kinematics

def pengzhuang(xa,ya,xb,yb,xc,yc,r,rod_radius):
    A = np.array([xa, ya], dtype=float)
    B = np.array([xb, yb], dtype=float)
    C = np.array([xc, yc], dtype=float)

    AB = B - A  # 从 A 指向 B
    AC = C - A  # 从 A 指向圆心 C
    if np.dot(AB, AB) < 1e-12:  # 检查线段长度是否为零
        P=A
    else:
        t=np.dot(AB, AC) / np.dot(AB, AB)  # 投影比例
        t = np.clip(t, 0, 1)  # 限制在 [0, 1] 范围内
        P = A + t * AB  # 线段上最近点
    distance = np.linalg.norm(P - C)  # 计算投影点到圆心的距离
    eps = 1e-12  # 单位 m，适用于当前米级实验
    if distance > r+ rod_radius+eps:
        return False
    else:
        return True

if __name__ == "__main__":
  a1 = 0.30
  a2=0.25
  q1 = np.deg2rad(30.0)
  q2 = np.deg2rad(45.0)
  O = np.array([0.0, 0.0])
  E = np.array([
        a1 * np.cos(q1),
        a1 * np.sin(q1),
   ])
  T = forward_kinematics(q1, q2, a1, a2)
  tip = T[:2, 3]  # 末端的二维坐标
  xc, yc = 0.15, 0.08
  r = 0.04
  rod_radius = 0.01

    # 第一杆：基座 O → 肘部 E
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
  print("第一杆碰撞" if collision_1 else "第一杆安全")
  print("第二杆碰撞" if collision_2 else "第二杆安全")
  print("机械臂碰撞" if collision else "机械臂安全")
