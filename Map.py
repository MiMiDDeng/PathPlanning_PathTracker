from common import *

# 地图读取
IMAGE_PATH = 'Path_to_your_jpg'  # 原图路径
HIGHT = 350  # 地图高度
WIDTH = 600  # 地图宽度

THRESH = 172  # 图片二值化阈值, 大于阈值的部分被置为255, 小于部分被置为0

MAP = GridMap(IMAGE_PATH, THRESH, HIGHT, WIDTH)  # 栅格地图对象

# 起点终点
START = (290, 270)  # 起点坐标 y轴向下为正
END = (200, 150)  # 终点坐标 y轴向下为正