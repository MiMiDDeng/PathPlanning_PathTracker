import numpy as np
import math

class Pure_pursuit:
    def __init__(self, b=0.5, look_ahead_distance=1.0):
        self.b = b  # 机器人轮距
        self.look_ahead_distance = look_ahead_distance  # 前视距离
        self.x = 0.0  # 初始位置x
        self.y = 0.0  # 初始位置y
        self.theta = 0.0  # 初始朝向角度
        self.x_traj = [self.x]  # 轨迹记录
        self.y_traj = [self.y]

    def set_robot_state(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.x_traj = [self.x]
        self.y_traj = [self.y]

    def calculate_control(self, path):
        closest_dist = float('inf')
        closest_point = None
        for point in path:
            dist = np.hypot(point[0] - self.x, point[1] - self.y)
            if dist < closest_dist:
                closest_dist = dist
                closest_point = point
        if closest_point is None:
            return 0.0, 0.0
        # 计算前视距离内的目标点
        target_dist = closest_dist + self.look_ahead_distance
        for i, point in enumerate(path):
            if np.hypot(point[0] - self.x, point[1] - self.y) > target_dist:
                break
        # 获取目标点
        if i == 0:
            target_point = path[0]
        else:
            target_point = path[i - 1]
        # 计算机器人朝向和目标点的方向
        alpha = np.arctan2(target_point[1] - self.y, target_point[0] - self.x) - self.theta
        # 计算角速度
        omega = 2 * np.sin(alpha) / self.look_ahead_distance
        # 计算轮速度
        v = 1.0
        # 计算左右轮速度
        left = v - self.b * omega / 2
        right = v + self.b * omega / 2
        return left, right

    def pure_pursuit_control(self, path):
        for i in range(len(path)):
            left, right = self.calculate_control(path[i:])
            v = (left + right) / 2
            omega = (right - left) / self.b
            self.x += v * np.cos(self.theta)
            self.y += v * np.sin(self.theta)
            self.theta += omega
            self.x_traj.append(self.x)
            self.y_traj.append(self.y)

    def get_traj(self):
        return self.x_traj, self.y_traj
