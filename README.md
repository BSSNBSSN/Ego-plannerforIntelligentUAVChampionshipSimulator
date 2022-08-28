# Ego-planner for IntelligentUAVChampionship

本仓库基于ZJU-Fast-Lab的[《从零制作自主空中机器人》](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)，在使用本仓库前，请先了解上述教程中的内容

## 使用
``` bash
cd $(Your catkin_ws path)

git clone git@github.com:BSSNBSSN/Ego-plannerforIntelligentUAVChampionshipSimulator.git

cd Ego-plannerforIntelligentUAVChampionshipSimulator

# 编译过程中如果出现错误属正常现象，连续编译三到四次一般就会解决
catkin build

echo "source $(Your catkin_ws path)/Ego-plannerforIntelligentUAVChampionshipSimulator/devel/setup.bash" >> ~/.bashrc
```

在三个终端中分别启动

比赛模拟器适用的rviz
``` bash
roslaunch reverser rviz.launch 
```
ego-planner启动文件
``` bash
roslaunch ego_planner single_run_in_exp.launch
```
ego-planner与比赛模拟器z轴方向相反，此包用于转换坐标系
``` bash
roslaunch reverser bringup.launch 
```
随后启动比赛模拟器，起飞后在rviz内设定目标点即可使飞机自主飞行



TODO：

- [x] 在模拟器中跑通ego-planner，对齐模拟器和ego-planner的坐标系
- [x] 使egoego-planner正确处理/move_base_simple/goal的z轴信息
- [ ] 识别障碍环
- [ ] 确定障碍环三维坐标
- [ ] 自动穿越障碍环
- [ ] 全流程自主穿越
- [ ] 穿越动态障碍环

