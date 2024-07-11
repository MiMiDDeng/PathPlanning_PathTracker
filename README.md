#参考：https://github.com/zhaohaojie1998/Path-Planning

#PathPlanning_PathTracker

#本项目实现了
（1）图片二值化，将输入的图片转化为具有障碍物的地图；
（2）PathPlanning：包括了DFS、BFS、Dijkstra、A_star算法；
（3）基于上述PathPlanning得到的路径的PathTracker：包括PID、Pure_pursuit算法；

#若要运行本代码，则在main文件中运行即可；若要修改初始点与目标点或者输入图片，则在Map文件中修改即可；若要修改PathPlanning与PathTracker算法，则
在main文件中修改即可；

#输入图片自行选择，也可以自己手绘；在Map文件中修改路径即可；运行代码之后，地图会保存在map.png文件中

#Python版本：3.10

#库需求：见requirements.txt；

#在本例中，Astar算法并不是最优，但是最快；BFS与Dijkstra最优但是很慢；BFS最烂但是较快。

![Output](https://github.com/MiMiDDeng/PathPlanning_PathTracker/assets/156818636/99d52db3-a35d-425d-856c-23f89aadae56)
