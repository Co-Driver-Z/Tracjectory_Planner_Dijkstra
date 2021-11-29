"""
    Dijkstra Grid
    @author: zz
    $ref: huiming zhou  from github（https://github.com/zhm-real）
"""

import math
import heapq
from Dijkstra_env_plot_zz import Env, Plotting


# Dijkstra算法
class Dijkstra:
    # Dijkstra算法参数初始化  点的形式 -> (x, y)
    # 时间复杂度： n * (2 + 8 * 1 * log n) = n log n
    # 空间复杂度； 4 * n = n
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.env = Env()
        self.mv = self.env.motions
        self.obs = self.env.obs
        self.plotting = Plotting(self.start, self.goal)
        self.CLOSED = []
        self.OPEN = []
        self.COST = dict()
        self.PARENT = dict()

    # Dijkstra算法寻路 @ 返回起点->终点的路程
    def searching(self):
        self.PARENT[self.start] = self.start
        self.COST[self.start] = 0
        heapq.heappush(self.OPEN, (0, self.start))
        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)           # 推出起点至OPEN中节点COST最小的节点，_ 应该为COST； s为 节点信息(x, y)
            self.CLOSED.append(s)
            if s == self.goal:                        # 首先先判断是否推出的终点！
                break
            for s_n in self.get_neighbor(s):          # 如果没推出终点，使用该点寻找邻接节点
                new_cost = _ + self.distance(s, s_n)
                if new_cost == math.inf:
                    continue
                elif s_n in self.COST and new_cost < self.COST[s_n]:
                    self.COST[s_n] = new_cost
                    self.PARENT[s_n] = s
                elif s_n not in self.COST and s_n not in self.CLOSED:
                    heapq.heappush(self.OPEN, (new_cost, s_n))
                    self.COST[s_n] = new_cost
                    self.PARENT[s_n] = s
        return self.extract_path(self.PARENT), self.CLOSED

    def get_neighbor(self, s):                        # 返回s节点的临接节点，总共8个
        return [(s[0] + mv[0], s[1] + mv[1]) for mv in self.mv]

    def distance(self, s_start, s_goal):              # 有障碍物，返回inf
        if self.is_collision(s_start, s_goal):
            return math.inf
        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):           # 判断是否有障碍物，有->true; 无->false;
        if s_start in self.obs or s_end in self.obs:
            return True
        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:    # 判断两点斜线分布
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
            if s1 in self.obs or s2 in self.obs:      # 如果斜线垂直方向有障碍物则视为有障碍物
                return True
        return False

    def extract_path(self, PARENT):                   # 返回路径
        path = [self.goal]
        s = self.goal
        while True:                                   # 从终点往回寻找父节点
            s = PARENT[s]                             # 直至找到起点：起点的父节点就是自身
            path.append(s)
            if s == self.start:
                break
        return list(path)


# 测试函数入口
def main():
    print("Hello World!")
    start = (5, 5)
    # goal = (45, 25)
    goal = (45, 5)
    dijkstra = Dijkstra(start, goal)                               # 算法初始化
    path, visited = dijkstra.searching()                           # 返回路径，和CLOSED容器->灰色点
    dijkstra.plotting.animation(path, visited, "Dijkstra")         # 产生图片


# GO！
if __name__ == '__main__':
    main()




