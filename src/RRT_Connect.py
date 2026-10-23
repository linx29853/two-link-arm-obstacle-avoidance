"""学习者 RRT-Connect 实验的函数封装，角度统一使用弧度。

保留原实验的最近邻、限步扩展、双树交替和路径回溯逻辑。
随机采样范围仍为两个关节各 ±150°。
模块名使用下划线，可在 src 位于模块搜索路径时导入。
"""

import numpy as np

from armlab.is_pose_valid import is_pose_valid
from armlab.edge_check import is_edge_valid


def extend_once(
    tree, parents, q_target, step_size,
    a1, a2, xc, yc, r, rod_radius,
    sample_count=11,
):
    """尝试扩展一次；成功返回节点索引，受阻返回 None。"""
    nearest_index = None
    nearest_distance = float("inf")
    for index, node in enumerate(tree):
        distance = np.linalg.norm(q_target - node)
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_index = index

    q_near = tree[nearest_index]
    direction = q_target - q_near
    distance = np.linalg.norm(direction)
    if distance < 1e-12:
        return nearest_index
    if distance <= step_size:
        q_new = q_target.copy()
    else:
        q_new = q_near + step_size * direction / distance

    edge_valid = is_edge_valid(
        q_near, q_new,
        sample_count=sample_count,
        a1=a1, a2=a2, xc=xc, yc=yc, r=r, rod_radius=rod_radius,
    )
    if edge_valid:
        tree.append(q_new.copy())
        parents.append(nearest_index)
        return len(tree) - 1
    else:
        return None


def connect(
    tree, parents, q_target, step_size,
    a1, a2, xc, yc, r, rod_radius,
    sample_count=11,
):
    """朝固定目标连续扩展；接通返回索引，受阻返回 None。"""
    result = extend_once(
        tree, parents, q_target, step_size,
        a1=a1, a2=a2, xc=xc, yc=yc, r=r, rod_radius=rod_radius,
        sample_count=sample_count,
    )
    while True:
        if result is None:
            return None
        elif np.linalg.norm(tree[result] - q_target) < 1e-12:
            return result
        else:
            result = extend_once(
                tree, parents, q_target, step_size,
                a1=a1, a2=a2, xc=xc, yc=yc, r=r, rod_radius=rod_radius,
                sample_count=sample_count,
            )


def rrt_connect(
    q_start, q_goal,
    a1, a2, xc, yc, r, rod_radius,
    step_size, sample_count=11,
    max_iterations=1000, seed=42,
):
    """搜索并返回弧度姿态列表；无效起终点或预算耗尽返回 None。

    q_start、q_goal 为长度为 2 的 NumPy 数组；step_size 为正弧度步长，
    sample_count 至少为 2。路径逐段复核由调用者执行。
    """
    rng = np.random.default_rng(seed)
    tree_start = [q_start.copy()]
    parents_start = [None]
    tree_goal = [q_goal.copy()]
    parents_goal = [None]

    start_valid = is_pose_valid(
        tree_start[0][0], tree_start[0][1],
        a1=a1, a2=a2, xc=xc, yc=yc, r=r, rod_radius=rod_radius,
    )
    goal_valid = is_pose_valid(
        tree_goal[0][0], tree_goal[0][1],
        a1=a1, a2=a2, xc=xc, yc=yc, r=r, rod_radius=rod_radius,
    )
    if not start_valid:
        return None
    if not goal_valid:
        return None

    if np.linalg.norm(q_start - q_goal) < 1e-12:
        return [q_start.copy()]

    found = False
    for attempt in range(max_iterations):
        if attempt % 2 == 0:
            tree_a, parents_a = tree_start, parents_start
            tree_b, parents_b = tree_goal, parents_goal
        else:
            tree_a, parents_a = tree_goal, parents_goal
            tree_b, parents_b = tree_start, parents_start
        index_a = extend_once(
            tree_a, parents_a,
            np.deg2rad(rng.uniform(-150, 150, size=2)), step_size,
            a1=a1, a2=a2, xc=xc, yc=yc, r=r, rod_radius=rod_radius,
            sample_count=sample_count,
        )
        if index_a is None:
            continue
        q_meet = tree_a[index_a]
        index_b = connect(
            tree_b, parents_b, q_meet, step_size,
            a1=a1, a2=a2, xc=xc, yc=yc, r=r, rod_radius=rod_radius,
            sample_count=sample_count,
        )

        if index_b is not None:
            found = True
            if attempt % 2 == 0:
                path_start = []
                index = index_a
                while index is not None:
                    path_start.append(tree_start[index])
                    index = parents_start[index]
                path_start.reverse()

                path_goal = []
                index = index_b
                while index is not None:
                    path_goal.append(tree_goal[index])
                    index = parents_goal[index]
            else:
                path_start = []
                index = index_b
                while index is not None:
                    path_start.append(tree_start[index])
                    index = parents_start[index]
                path_start.reverse()

                path_goal = []
                index = index_a
                while index is not None:
                    path_goal.append(tree_goal[index])
                    index = parents_goal[index]
            break

    if found:
        path = path_start + path_goal[1:]
        return path
    else:
        return None
