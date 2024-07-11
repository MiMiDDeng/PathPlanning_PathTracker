from functools import lru_cache
from common import *
from Map import *

Queue_Type = 0

""" ---------------------------- A*算法 ---------------------------- """


# 设置OpenList使用的优先队列
if Queue_Type == 0:
    NodeQueue = SetQueue
elif Queue_Type == 1:
    NodeQueue = ListQueue
else:
    NodeQueue = PriorityQueuePro


# A*算法
class AStar:
    """A*算法"""

    def __init__(
            self,
            start_pos=START,
            end_pos=END,
            map_array=MAP.map_array,
            move_step=3,
            move_direction=8,
    ):
        # 网格化地图
        self.map_array = map_array  # H * W
        self.width = self.map_array.shape[1]
        self.high = self.map_array.shape[0]
        # 起点终点
        self.start = Node(*start_pos)  # 初始位置
        self.end = Node(*end_pos)  # 结束位置
        # Error Check
        if not self._in_map(self.start) or not self._in_map(self.end):
            raise ValueError(f"x坐标范围0~{self.width - 1}, y坐标范围0~{self.height - 1}")
        if self._is_collided(self.start):
            raise ValueError(f"起点x坐标或y坐标在障碍物上")
        if self._is_collided(self.end):
            raise ValueError(f"终点x坐标或y坐标在障碍物上")
        # 算法初始化
        self.reset(move_step, move_direction)
    def reset(self, move_step=3, move_direction=8):
        """重置算法"""
        self.__reset_flag = False
        self.move_step = move_step  # 移动步长(搜索后期会减小)
        self.move_direction = move_direction  # 移动方向 8 个
        self.close_set = set()  # 存储已经走过的位置及其G值
        self.open_queue = NodeQueue()  # 存储当前位置周围可行的位置及其F值
        self.path_list = []  # 存储路径(CloseList里的数据无序)
    def search(self):
        return self.__call__()
    def _in_map(self, node: Node):
        return (0 <= node.x < self.width) and (0 <= node.y < self.high)  # 右边不能取等!!!
    def _is_collided(self, node: Node):
        return self.map_array[node.y, node.x] == 0
    def _move(self):
        @lru_cache(maxsize=3)  # 避免参数相同时重复计算
        def _move(move_step: int, move_direction: int):
            move = [
                (0, move_step),  # 上
                (0, -move_step),  # 下
                (-move_step, 0),  # 左
                (move_step, 0),  # 右
                (move_step, move_step),  # 右上
                (move_step, -move_step),  # 右下
                (-move_step, move_step),  # 左上
                (-move_step, -move_step),  # 左下
            ]
            return move[0:move_direction]  # 坐标增量
        return _move(self.move_step, self.move_direction)
    def _update_open_list(self, curr: Node):
        """open_list添加可行点"""
        for add in self._move():
            # 更新节点
            next_ = curr + add  # x、y、cost、parent都更新了

            # 新位置是否在地图外边
            if not self._in_map(next_):
                continue
            # 新位置是否碰到障碍物
            if self._is_collided(next_):
                continue
            # 新位置是否在 CloseList 中
            if next_ in self.close_set:
                continue
            # 把节点的 G 代价改成 F 代价
            H = next_ - self.end
            next_.cost += H
            self.open_queue.put(next_)
            # 当剩余距离小时, 走慢一点
            if H < 20:
                self.move_step = 1
    def __call__(self):
        """A*路径搜索"""
        assert not self.__reset_flag, "call之前需要reset"
        print("搜索中\n")
        # 初始化 OpenList
        self.open_queue.put(self.start)

        # 正向搜索节点
        tic()
        while not self.open_queue.empty():
            # 弹出 OpenList 代价 F 最小的点
            curr = self.open_queue.get()  # OpenList里是 F
            curr.cost -= (curr - self.end)  # G = F - H
            # 更新 OpenList
            self._update_open_list(curr)
            # 更新 CloseList
            self.close_set.add(curr)
            # 结束迭代
            if curr == self.end:
                break
        print("路径搜索完成\n")
        toc()

        # 节点组合成路径
        while curr.parent is not None:
            self.path_list.append(curr)
            curr = curr.parent
        self.path_list.reverse()

        # 需要重置
        self.__reset_flag = True

        return self.path_list