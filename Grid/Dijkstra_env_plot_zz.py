"""
    Dijkstra Grid
    @author: zz
    $ref: huiming zhou  from github（https://github.com/zhm-real）
"""

import matplotlib.pyplot as plt


class Env:
    def __init__(self):
        self.x_range = 51  # size of background
        self.y_range = 32  # size of background
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),      # 8个方向前进，从最左侧顺时针旋转
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()                              # 获取障碍物地图

    def obs_map(self):
        x = self.x_range
        y = self.y_range
        obs = set()
        for i in range(x):                                     # 设置边界
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))
        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        for i in range(10, 21):                                # 设置其他障碍物
            obs.add((i, 15))
        for i in range(15, 31):
            obs.add((20, i))
        for i in range(0, 15):
            obs.add((30, i))
        for i in range(16, 31):
            obs.add((40, i))

        # for i in range(10, 21):                              # 设置其他障碍物
        #     obs.add((i, 20))
        # for i in range(20):
        #     obs.add((20, i))
        # for i in range(5, 31):
        #     obs.add((30, i))
        # for i in range(27):
        #     obs.add((40, i))
        return obs


class Plotting:
    def __init__(self, xI, xG):                     # xI -> s_start
        self.xI, self.xG = xI, xG                   # xG -> s_goal
        self.env = Env()
        self.obs = self.env.obs_map()

    def animation(self, path, visited, name):       # 正式绘图的启动函数
        self.plot_grid(name)
        self.plot_visited(visited)
        self.plot_path(path)
        plt.show()

    def plot_grid(self, name):                      # 绘图 -> obs start goal
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")

    def plot_visited(self, visited, cl='gray'):     # 绘图 -> 灰色
        if self.xI in visited:
            visited.remove(self.xI)
        if self.xG in visited:
            visited.remove(self.xG)
        count = 0
        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker='o')

    def plot_path(self, path, cl='r', flag=False):  # 绘图 -> 路径
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        if not flag:
            plt.plot(path_x, path_y, linewidth='3', color='r')
        else:
            plt.plot(path_x, path_y, linewidth='3', color=cl)
        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")
