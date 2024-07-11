from A_star import *
from Dijkstra import *
from DFS import *
from BFS import *
from PID import *
from Pure_pursuit import *
from Map import *

# 提取路径函数
def extract_path(path):
    new_path = []
    for node in path:
        new_path.append( (node.x,node.y) )
    return new_path

if __name__ == '__main__':
    #路径规划
    path = AStar()()
    '''
    path = Astar()()
    path = Dijkstra()()
    path = DFS()()
    path = BFS()()
    '''

    path=extract_path(path)

    #路径跟踪


    #PID

    temp_path = path_subdivision(path,END,dt=0.5)
    print(len(temp_path))
    pid=PID(dt=0.5)
    pid.set_state(*START, 0.0)
    pid.set_pid_param(kp=2.0, ki=0.1, kd=0.1)
    pid.PID_control(temp_path)
    x_traj_1, y_traj_1 = pid.get_traj()
    new_path_1=[]
    for i in range(len(x_traj_1)):
        new_path_1.append([x_traj_1[i],y_traj_1[i]])
    '''

    #Pure_pursuit
    temp_path = path_subdivision(path, END, dt=1.2)
    pure_pursuit=Pure_pursuit()
    pure_pursuit.set_robot_state(*START,0.0)
    pure_pursuit.pure_pursuit_control(temp_path)
    x_traj_2,y_traj_2=pure_pursuit.get_traj()
    new_path_2 = []
    for i in range(len(x_traj_2)):
        new_path_2.append([x_traj_2[i], y_traj_2[i]])
    '''

    #绘图
    MAP.show_path(path,new_path_1)