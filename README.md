# 用于仿真与实际的跑动效果对比
**由于时间关系，仅使用rviz进行可视化**
## 依赖
```
ros
sudo apt install python-catkin-tools
```
## 编译方法
```
git clone https://github.com/Feng1909/sim_and_real.git
cd ~/sim_and_real
catkin_make
```
## 使用步骤
* 实车跑动，并录制vive数据和车辆传感器数据
* 运行
    ```
    cd ~/sim_and_real/src/visualier  
    python3 merge.py name_1.bag name_2.bag name_3.bag
    # name_1 name_2 name_3分别是vive数据集、车辆传感器数据集、合并后自定义名称的数据集，此步骤用于合并俩数据集
    ```
* 运行`roslaunch visual view.launch`，此命令会打开rviz
界面并且自动加载rviz文件夹下的rviz配置文件
* 运行`rosbag play name_3.bag -l`，一定要加`-l`命令，程序会根据时间戳将本轮数据的位置信息记录为`path.txt`，存储于`.ros`文件夹下
* 待数据集跑了一遍后，关闭上述程序，运行`roslaunch control control.launch`，此程序会根据仿真中车辆的里程计信息计算车辆的位置，并且同时控制小车根据保存的路径前进。在`control.cpp`的`odom_Callback`函数中对车的位置进行了粗略的修正(`state_x state_y state_theta`)，具体的数值需要根据实际情况进行更改，vive的坐标系已转化为前x，左y，与仿真相同