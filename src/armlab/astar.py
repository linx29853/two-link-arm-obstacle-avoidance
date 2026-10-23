import numpy as np
import heapq
from armlab.is_pose_valid import is_pose_valid
from armlab.edge_check import is_edge_valid

def node_to_angles(node, angles_deg):
    """将 (q2 行号, q1 列号) 转为 [q1, q2] 弧度。"""
    j, i = node
    return np.deg2rad([angles_deg[i], angles_deg[j]])


def build_valid_grid(angles_deg, a1, a2, xc, yc, r, rod_radius):
    """扫描方形关节网格，True 表示姿态有效。"""
    n = len(angles_deg)
    valid_grid = np.zeros((n, n), dtype=bool)
    for j in range(n):
        for i in range(n):
            q1, q2 = node_to_angles((j, i), angles_deg)
            valid_grid[j, i] = is_pose_valid(
                q1, q2, a1, a2, xc, yc, r, rod_radius
            )
    return valid_grid


def neighbors(node, grid_size):
    """返回没有越界的四邻居，尚未检查碰撞。"""
    j, i = node  # 行是 q2，列是 q1

    candidates = [
        (j - 1, i),  # 下
        (j + 1, i),  # 上
        (j, i - 1),  # 左
        (j, i + 1),  # 右
    ]

    return [
        (nj, ni)
        for nj, ni in candidates
        if 0 <= nj < grid_size and 0 <= ni < grid_size
    ]

def traversable_neighbors(
    node, valid_grid, angles_deg,
    a1, a2, xc, yc, r, rod_radius, sample_count=76,
):
    """返回姿态有效且连接通过离散采样检查的邻居。"""
    n = valid_grid.shape[0]

    if not valid_grid[node]:
        return []

    valid_neighbors = []
    q = node_to_angles(node, angles_deg)
    for nj, ni in neighbors(node, n):
        if not valid_grid[nj, ni]:
            continue
        q_neighbor = node_to_angles((nj, ni), angles_deg)
        edge_valid = is_edge_valid(
            q, q_neighbor,
            sample_count=sample_count,
            a1=a1, a2=a2, xc=xc, yc=yc,
            r=r, rod_radius=rod_radius,
        )
        if edge_valid:
            valid_neighbors.append((nj, ni))
    return valid_neighbors

def astar_search(
    start, goal, valid_grid, angles_deg,
    a1, a2, xc, yc, r, rod_radius,
    sample_count=76,
):
    if not valid_grid[start]:
        print("搜索失败：起点姿态无效")
        return None

    if not valid_grid[goal]:
        print("搜索失败：目标姿态无效")
        return None
    q_goal = node_to_angles(goal, angles_deg)
    print("有效性：",valid_grid[goal])
    # 起终点有效性检查

    g={start:0.0}
    parent={start:None}
    q_start = node_to_angles(start, angles_deg)
    open_heap = []
    h_start = np.linalg.norm(q_goal - q_start)
    heapq.heappush(open_heap, (g[start] + h_start, start))
    found = False
    # g、parent、open_heap、found 的初始化

    while open_heap:
        best_f, current = heapq.heappop(open_heap)
        q_current = node_to_angles(current, angles_deg)
        h_current = np.linalg.norm(q_goal - q_current)
        latest_f = g[current] + h_current

        if best_f > latest_f:
            print("跳过过期记录：", current)
            continue
        if current == goal:
            found = True 
            print("找到目标：", current)
            break
            
        print("下一步探索：", current, "f =", best_f)
    
        current_neighbors = traversable_neighbors(
            current, valid_grid, angles_deg,
            a1, a2, xc, yc, r, rod_radius,
            sample_count=sample_count,
        )
        print("待扩展节点：", current)
        print("它的可通行邻居：", current_neighbors)
        for neighbor in current_neighbors:
            q_neighbor = node_to_angles(neighbor, angles_deg)
            edge_cost = np.linalg.norm(q_neighbor - q_current)
            tentative_g = g[current] + edge_cost
    
            print(
                f"邻居 {neighbor}:"
                f"旧代价={g.get(neighbor, float('inf')):.3f},"
                f"新代价={tentative_g:.3f}"
            )
            if tentative_g < g.get(neighbor, float("inf")):
                g[neighbor] = tentative_g
                parent[neighbor] = current
                h = np.linalg.norm(q_goal - q_neighbor)
                f = g[neighbor] + h
                heapq.heappush(open_heap, (f, neighbor))
                print(f"更新 {neighbor}：g={g[neighbor]:.3f}，来源={current}")
            else:
                print(f"不更新 {neighbor}：新路线没有更短")
        print("当前等待队列（按 f 排序）：")
        for f_value, node in sorted(open_heap):
            print(f"节点 {node}：f={f_value:.3f} ")
        print("\n ----------------------------------------")
    print("代价：", g)
    print("来源：", parent)
    path = []
    # while open_heap 搜索循环

    if found:
        node = goal
        while node is not None:
            path.append(node)
            node = parent[node]

        path.reverse()
        return path
    else:
        return None
    # 搬入路径回溯