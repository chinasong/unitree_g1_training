
一、说明：

环境要求：ubuntu20.04 ros1 noetic
如果是canda环境请先退出！！conda deactivate或暂时在.bashhrc里注释掉conda的自启动。

1、编译：

 Compile and install the Livox-SDK2:
```shell
$ cd catkin_ws/Livox-SDK/
$ mkdir build
$ cd build
$ cmake .. && make -j
$ sudo make install
$ cd catkin_ws/
$ source /opt/ros/noetic/setup.bash
$ catkin_make
``

2、运行建图
建图：

运行算法：

source devel/setup.bash

roslaunch fast_lio mapping_mid360.launch

数据播放：

rosbag play map.bag

3、运行定位：
定位：
运行算法：

source devel/setup.bash

roslaunch mid360_locate localization.launch

数据播放：

rosbag play localization_test.bag

二、作业要求：
1、修改建图代码，构建z轴朝上的地图。
2、修改定位代码，使用你自己建的地图替换原有地图,还得调整配置和初始化，点云z轴朝上。

