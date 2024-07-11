import numpy as np
import math

class PID:
    #初始化机器人
    def __init__(self, b=0.5, dt=0.1, kv=1.0, kp=1.0, ki=0.1, kd=0.01, look_ahead_distance=1.0):
        self.set_robot_param(b, dt)
        self.set_pid_param(kv, kp, ki, kd)
        self.set_pure_param(look_ahead_distance)
        self.set_state()
        self.integral_error = 0.0
        self.previous_error = 0.0

    #初始化机器人长度与时间步长
    def set_robot_param(self, b=0.5, dt=0.1):
        self.b = b
        self.dt = dt

    #初始化机器人的初始位置和朝向角度
    def set_state(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.x_traj = [self.x]
        self.y_traj = [self.y]

    #初始化PID控制器的参数
    def set_pid_param(self, kv=1.0, kp=1.0, ki=0.1, kd=0.01):
        self.kv = kv
        self.kp = kp
        self.ki = ki
        self.kd = kd

    #初始化pure算法的前视距离
    def set_pure_param(self, look_ahead_distance):
        self.look_ahead_distance = look_ahead_distance

    #PID控制计算方法
    def calculate_control(self, path):
        '''根据机器人当前位置和给定路径，计算到路径上最近点的距离误差(e_y)和角度误差(e_theta),使用PID控制算法，计算出左右轮的速度控制量
        返回计算得到的左右轮速度'''
        closest_point = min(path, key=lambda p:np.hypot(p[0]-self.x, p[1]-self.y))
        x_d, y_d = closest_point
        # 计算误差
        e_y = np.hypot(x_d - self.x, y_d - self.y)
        e_theta = np.arctan2(y_d - self.y, x_d - self.x) - self.theta
        # PID控制
        self.integral_error += e_theta * self.dt
        derivative_error = (e_theta - self.previous_error) / self.dt
        self.previous_error = e_theta
        v = self.kv * e_y
        omega = self.kp * e_theta + self.ki * self.integral_error + self.kd * derivative_error
        # 计算左右轮速度
        left = v - self.b * omega / 2
        right = v + self.b * omega / 2
        return left, right

    #PID控制方法
    '''使用PID控制算法来驱动机器人沿着给定路径移动,循环遍历路径上的每个点
    计算每个点处的控制量，更新机器人的位置和朝向,将每个时间步的位置记录到轨迹中，直到机器人接近路径的终点'''
    def PID_control(self, path):
        for i in range(len(path)):
            left, right = self.calculate_control(path[i:])
            # 更新机器人状态
            v = (left + right) / 2
            omega = (right - left) / self.b
            self.x += v * np.cos(self.theta) * self.dt
            self.y += v * np.sin(self.theta) * self.dt
            self.theta += omega * self.dt
            self.x_traj.append(self.x)
            self.y_traj.append(self.y)
        # 与终点存在一定距离时，继续控制
        self.end_control(goal=path[-1])

    def end_control(self, goal, threshold=0.05):
        """
        PID中控制机器人到达终点
        :param goal: 终点坐标
        :param threshold: 到达终点的阈值
        """
        print(np.hypot(goal[0] - self.x, goal[1] - self.y))
        while np.hypot(goal[0]-self.x, goal[1]-self.y) > threshold:
            left, right = self.calculate_control([goal])
            v = (left + right) / 2
            omega = (right - left) / self.b
            self.x += v * np.cos(self.theta) * self.dt
            self.y += v * np.sin(self.theta) * self.dt
            self.theta += omega * self.dt

            self.x_traj.append(self.x)
            self.y_traj.append(self.y)


    def get_traj(self):
        return self.x_traj, self.y_traj

def path_subdivision(path, goal, dt=0.1):
    new_path = []
    for i in range(len(path) - 1):
        distance = math.sqrt((path[i][0] - path[i + 1][0]) ** 2 + (path[i][1] - path[i + 1][1]) ** 2)
        num = math.ceil(distance / dt)
        x = np.linspace(path[i][0], path[i + 1][0], num)
        y = np.linspace(path[i][1], path[i + 1][1], num)
        new_path.extend(list(zip(x, y)))
    new_path.append(goal)
    return new_path