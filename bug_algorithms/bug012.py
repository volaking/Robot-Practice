import numpy as np
import matplotlib.pyplot as plt

show_animation = True
# 可以改进的点，目前的normal模式是沿45度角走，应该修改为朝向target走
class BugPlanner:
    def __init__(self, start_x, start_y, goal_x, goal_y, obs_x, obs_y):
        self.goal_x, self.goal_y = goal_x, goal_y
        self.obs_x, self.obs_y = obs_x, obs_y
        self.r_x, self.r_y = [start_x], [start_y]
        self.out_x, self.out_y = [], []

        # 计算障碍物外围边界的坐标

        for o_x, o_y in zip(obs_x, obs_y):
            for add_x, add_y in zip([1, 0, -1, -1, -1, 0, 1, 1],
                                    [1, 1, 1, 0, -1, -1, -1, 0]):
                cand_x, cand_y = o_x + add_x, o_y + add_y
                valid_point = True
                for _x, _y in zip(obs_x, obs_y):
                    if cand_x == _x and cand_y == _y:
                        valid_point = False
                        break
                if valid_point:
                    self.out_x.append(cand_x), self.out_y.append(cand_y)

    
    # 正常模式下，下一步的坐标

    def mov_normal(self):
        return self.r_x[-1] + np.sign(self.goal_x - self.r_x[-1]), self.r_y[-1] + np.sign(self.goal_y - self.r_y[-1])

    
    # 避障模式下，下一步的坐标  
    def mov_to_next_obs(self, visited_x, visited_y):
        for add_x, add_y in zip([1, 0, -1, 0], [0, 1, 0, -1]):
            c_x, c_y = self.r_x[-1] + add_x, self.r_y[-1] + add_y
            for _x, _y in zip(self.out_x, self.out_y):
                use_pt = True
                if c_x == _x and c_y == _y: # 判断下一步是否位于障碍物边界
                    for v_x, v_y in zip(visited_x, visited_y):
                        if c_x == v_x and c_y == v_y:   # 如果位于障碍物边界，判断下一步是否已经走过
                            use_pt = False
                    if use_pt:  # 如果下一步位于障碍物边界，但是并没有走过，则继续沿障碍物边界运动
                        return c_x, c_y, False
                if not use_pt:  # 如果下一步位于障碍物边缘，而且已经走过，则继续检查其他方向
                    break    
        return self.r_x[-1], self.r_y[-1], True # 如果所有方向都已经走过或没有位于障碍物边界，则返回现在的位置，并返回True标记


    def bug0(self):
        # 如何让机器人沿斜线运动
        # 设置机器人的初始运动状态为正常模式

        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")
            plt.grid(True)
            plt.title('BUG 0')

        # 检查机器人初始位置，位于障碍物边界，则机器人设置为避障模式

        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == x_ob and self.r_y[-1] == y_ob:
                mov_dir = 'obs'
                break

        # 机器人开始运动

        visited_x, visited_y = [], []
        while True:

            # 循环终止条件，判断是否抵达终点

            if self.r_x[-1] == self.goal_x and \
                    self.r_y[-1] == self.goal_y:
                break

            # 根据机器人状态进行不同的运动方式

            if mov_dir == 'obs':
                cand_x, cand_y, _ = self.mov_to_next_obs(visited_x, visited_y)
                can_go_normal = True
                for x_ob, y_ob in zip(self.obs_x, self.obs_y):
                    if self.mov_normal()[0] == x_ob and \
                            self.mov_normal()[1] == y_ob:   # 如果机器人下一步执行正常运动，可能到达障碍物边界则不能切换模式
                        can_go_normal = False
                        break
                if can_go_normal:   # 判断是否可以切换为正常模式
                    mov_dir = 'normal'
                else:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)
                    visited_x.append(cand_x), visited_y.append(cand_y)
            elif mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    if cand_x == x_ob and cand_y == y_ob:   # 判断下一步的位置是否位于障碍物边缘，切换到避障模式
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x)
                    self.r_y.append(cand_y)

            if show_animation:
                plt.plot(self.r_x, self.r_y, "-r")
                plt.pause(0.001)
        
        if show_animation:
            plt.show()

    # bug1与bug0的主要区别是切换运动状态的模式不同，
    # bug0在遇到障碍物时切换为避障模式，在可以向目标前进时切换为正常模式
    # bug1在遇到障碍物时进入避障模式，在绕障一周后选择最接近目标点的位置出账

    def bug1(self):
        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf
        exit_x, exit_y = -np.inf, -np.inf   # 新增退出位置
        dist = np.inf                       # 新增dist
        back_to_start = False               # 机器人是否绕障碍一周回到原点
        second_round = False                # 机器人是否正在第二圈运动
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")
            plt.grid(True)
            plt.title('BUG 1')
        
        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == x_ob and self.r_y[-1] == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            if self.r_x[-1] == self.goal_x and \
                    self.r_y[-1] == self.goal_y:
                break
            

            # 判断机器人运动状态及状态转换

            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        mov_dir = 'obs'
                        dist = np.inf
                        back_to_start = False
                        second_round = False
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)

            # 机器人出障检测

            elif mov_dir == 'obs':
                cand_x, cand_y, back_to_start = \
                    self.mov_to_next_obs(visited_x, visited_y)
                # 计算障碍物周围距离目标的最小距离作为退出点
                d = np.linalg.norm(np.array([cand_x, cand_y] - np.array([self.goal_x, self.goal_y])))
                if d < dist and not second_round:
                    exit_x, exit_y = cand_x, cand_y
                    dist = d
                # 进入第二圈
                if back_to_start and not second_round:
                    second_round = True
                    del self.r_x[-len(visited_x):]
                    del self.r_y[-len(visited_y):]
                    visited_x[:], visited_y[:] = [], []
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                visited_x.append(cand_x), visited_y.append(cand_y)
                # 到达退出点切换为normal模式
                if cand_x == exit_x and \
                        cand_y == exit_y and \
                        second_round:
                    mov_dir = 'normal'
            if show_animation:
                plt.plot(self.r_x, self.r_y, '-r')
                plt.pause(0.001)
        if show_animation:
            plt.show()

    def bug2(self):
        mov_dir = 'normal'
        cand_x, cand_y = -np.inf, -np.inf
        if show_animation:
            plt.plot(self.obs_x, self.obs_y, ".k")
            plt.plot(self.r_x[-1], self.r_y[-1], "og")
            plt.plot(self.goal_x, self.goal_y, "xb")
            plt.plot(self.out_x, self.out_y, ".")

        straight_x, straight_y = [self.r_x[-1]], [self.r_y[-1]]
        hit_x, hit_y = [], []
        while True:
            if straight_x[-1] == self.goal_x and \
                    straight_y[-1] == self.goal_y:
                break
            c_x = straight_x[-1] + np.sign(self.goal_x - straight_x[-1])
            c_y = straight_y[-1] + np.sign(self.goal_y - straight_y[-1])
            for x_ob, y_ob in zip(self.out_x, self.out_y):
                if c_x == x_ob and c_y == y_ob:
                    hit_x.append(c_x), hit_y.append(c_y)
                    break
            straight_x.append(c_x), straight_y.append(c_y)
        if show_animation:
            plt.plot(straight_x, straight_y, ",")
            plt.plot(hit_x, hit_y, "d")
            plt.grid(True)
            plt.title('BUG 2')

        for x_ob, y_ob in zip(self.out_x, self.out_y):
            if self.r_x[-1] == x_ob and self.r_y[-1] == y_ob:
                mov_dir = 'obs'
                break

        visited_x, visited_y = [], []
        while True:
            if self.r_x[-1] == self.goal_x \
                    and self.r_y[-1] == self.goal_y:
                break
            if mov_dir == 'normal':
                cand_x, cand_y = self.mov_normal()
            if mov_dir == 'obs':
                cand_x, cand_y, _ = self.mov_to_next_obs(visited_x, visited_y)
            if mov_dir == 'normal':
                found_boundary = False
                for x_ob, y_ob in zip(self.out_x, self.out_y):
                    if cand_x == x_ob and cand_y == y_ob:
                        self.r_x.append(cand_x), self.r_y.append(cand_y)
                        visited_x[:], visited_y[:] = [], []
                        visited_x.append(cand_x), visited_y.append(cand_y)
                        del hit_x[0]
                        del hit_y[0]
                        mov_dir = 'obs'
                        found_boundary = True
                        break
                if not found_boundary:
                    self.r_x.append(cand_x), self.r_y.append(cand_y)
            elif mov_dir == 'obs':
                self.r_x.append(cand_x), self.r_y.append(cand_y)
                visited_x.append(cand_x), visited_y.append(cand_y)
                for i_x, i_y in zip(range(len(hit_x)), range(len(hit_y))):
                    if cand_x == hit_x[i_x] and cand_y == hit_y[i_y]:
                        del hit_x[i_x]
                        del hit_y[i_y]
                        mov_dir = 'normal'
                        break
            if show_animation:
                plt.plot(self.r_x, self.r_y, "-r")
                plt.pause(0.001)
        if show_animation:
            plt.show()

                

def main(bug_0, bug_1, bug_2):
    # set obstacle positions
    o_x, o_y = [], []

    s_x = 0.0
    s_y = 0.0
    g_x = 167.0
    g_y = 50.0

    for i in range(20, 40):
        for j in range(20, 40):
            o_x.append(i)
            o_y.append(j)

    for i in range(60, 100):
        for j in range(40, 80):
            o_x.append(i)
            o_y.append(j)

    for i in range(120, 140):
        for j in range(80, 100):
            o_x.append(i)
            o_y.append(j)

    for i in range(80, 140):
        for j in range(0, 20):
            o_x.append(i)
            o_y.append(j)

    for i in range(0, 20):
        for j in range(60, 100):
            o_x.append(i)
            o_y.append(j)

    for i in range(20, 40):
        for j in range(80, 100):
            o_x.append(i)
            o_y.append(j)

    for i in range(120, 160):
        for j in range(40, 60):
            o_x.append(i)
            o_y.append(j)

    if bug_0:
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y)
        my_Bug.bug0()
    if bug_1:
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y)
        my_Bug.bug1()
    if bug_2:
        my_Bug = BugPlanner(s_x, s_y, g_x, g_y, o_x, o_y)
        my_Bug.bug2()


if __name__ == '__main__':
    main(bug_0=False, bug_1=True, bug_2=False)
