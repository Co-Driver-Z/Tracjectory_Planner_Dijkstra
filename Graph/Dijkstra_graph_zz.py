"""
    Dijkstra G = (V, E)
    @author: zz
    $ref: https://blog.csdn.net/yalishadaa/article/details/55827681?ops_request_misc=
    %257B%2522request%255Fid%2522%253A%2522162709285316780261948754%2522%252C%2522scm
    %2522%253A%252220140713.130102334..%2522%257D&request_id=162709285316780261948754
    &biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~
    default-7-55827681.first_rank_v2_pc_rank_v29&utm_term=Dijkstra%E7%AE%97%E6%B3%95&
    spm=1018.2226.3001.4187
"""


import numpy as np
import pandas as pd
import math
import heapq


# 环境矩阵初始化
class Env:
    def __init__(self, env_initial, env_para):
        index = columns = env_para                    # 设置行列索引
        self.G = pd.DataFrame(np.eye(7, 7)*(-1), index=index, columns=columns)   # 环境矩阵初始化
        for index, row in env_initial.iterrows():     # 将环境参数变换到环境矩阵
            self.G.loc[row[0], row[1]] = row[2]
        self.G = self.G + self.G.T                    # 对.G进行转置 并合并
        self.G[self.G == 0] = math.inf                # 初始化无穷距离
        self.G[self.G == -2] = 0                      # 初始化自身距离


# Dijkstra算法
class Dijkstra:
    # Dijkstra算法参数初始化
    # 时间复杂度：n * (lgn + n) = n*lgn + n^2
    # 空间复杂度：4 * n = n
    def __init__(self, env):
        self.CLOSED = []         # 已确定最短路径的闭区间
        self.OPEN = []           # 未确定最短路径的开区间
        self.COST = dict()       # start~@的cost
        self.PARENT = dict()     # 抵达@的上一个节点
        self.env_G = env.G       # 获取环境矩阵
        print(self.env_G, '\n')

    # Dijkstra算法寻路 @ 返回起点->终点的路程
    def searching_result_return(self, v_start, v_goal):
        heapq.heappush(self.CLOSED, (0, v_start))     # 将出发点推入.CLOSED
        self.PARENT[v_start] = v_start
        self.COST[v_start] = 0
        action_initial = self.env_G.loc[v_start, :]
        for index, item in action_initial.drop(v_start, axis=0).iteritems():
            heapq.heappush(self.OPEN, (item, index))  # 将其他点推入.OPEN
            if item != math.inf:
                self.PARENT[index] = v_start

        while self.OPEN:
            dis, s = heapq.heappop(self.OPEN)      # 从.OPEN中推出dis最小的元组(在.OPEN中删除)
            heapq.heappush(self.CLOSED, (dis, s))  # 将最小的dis点推入.CLOSED
            self.COST[s] = dis                     # 设置到达s的最短dis
            if s == v_goal:                        # 到达终点，结束
                break
            open_temp = self.OPEN.copy()           # 暂存.OPEN
            for s_open in self.OPEN:
                if self.env_G.loc[s, s_open[1]] != math.inf and self.env_G.loc[s, s_open[1]] + self.COST[s] < s_open[0]:
                    open_temp.remove(s_open)
                    heapq.heappush(open_temp, (self.env_G.loc[s, s_open[1]] + self.COST[s], s_open[1]))
                    self.PARENT[s_open[1]] = s
                if self.env_G.loc[s, s_open[1]] != math.inf and self.env_G.loc[s, s_open[1]] + self.COST[s] == s_open[0]:
                    parent_temp_list = list(self.PARENT[s_open[1]])
                    parent_temp_list.append(s)
                    self.PARENT[s_open[1]] = parent_temp_list
            self.OPEN = open_temp.copy()           # 完成对.OPEN的更新

        # 返回起点->终点的路程
        result = []
        for i in self.s_parent(v_goal):
            result.append(i[:-2])                  # 去除多余的'-> '
        print(v_start + ' -> ' + v_goal + ' 的最短路径为: ', result)
        print('dis: ', self.COST[v_goal])
        return result                              # 返回结果

    # 递归找父节点
    def s_parent(self, v_goal):
        s_type = type(self.PARENT[v_goal])
        if s_type == str:
            if self.PARENT[v_goal] == v_goal:
                return [self.PARENT[v_goal] + '->']
            else:
                ret = self.s_parent(self.PARENT[v_goal])
                return [i + v_goal + '->' for i in ret]
        else:
            ret = []
            for key in self.PARENT[v_goal]:
                ret.extend([prev + v_goal + '->' for prev in self.s_parent(key)])
            return ret


# 测试函数入口
def main():
    env = pd.DataFrame([['A', 'B', 12],             # 环境信息
                        ['A', 'F', 16],
                        ['A', 'G', 14],
                        ['B', 'C', 10],
                        ['B', 'F', 7],
                        ['C', 'D', 3],
                        ['C', 'E', 5],
                        ['C', 'F', 6],
                        ['D', 'E', 4],
                        ['E', 'F', 2],
                        ['E', 'G', 8],
                        ['F', 'G', 9]])
    env_para = ['A', 'B', 'C', 'D', 'E', 'F', 'G']  # 节点列表
    start = 'D'                                     # 给出起点
    goal = 'B'                                      # 给出终点

    env = Env(env, env_para)                        # 获得环境矩阵
    dijkstra = Dijkstra(env)                        # Dijkstra算法初始化
    result = dijkstra.searching_result_return(start, goal)   # 得出结果


# GO！
if __name__ == '__main__':
    main()
