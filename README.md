## 简介

使用 urdf 文件构建仿真机器人，在 gazebo 搭建仿真环境，导入人造卫星的 stl 文件，利用仿真 kinect 深度相机获取人造卫星的点云数据并在 RVIZ 中可视化，将 PointCloud2 数据类型转化为 pcd 文件。

## 使用指南

进入目录

```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
```

下载软件包

```bash
git clone https://github.com/yym68686/ros-slam.git
```

回到根目录编译

```
cd ~/catkin_ws
catkin_make
```

让 ROS 找到软件包

```bash
source ~/catkin_ws/devel/setup.bash
```

启动 ROS 内核

```
roscore
```

结合 URDF 打开 gazebo 仿真环境

```bash
roslaunch ros-slam env.launch
```

打开机器人深度相机 kinect 等传感器并在 rviz 可视化

```bash
roslaunch ros-slam sensor.launch
```

打开机器人运动控制

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

按照提示使用键盘控制小车方向。运动控制需要下载依赖

```bash
sudo apt-get install -y ros-noetic-teleop-twist-keyboard
```

订阅 kinect 点云话题转化为点云 pcd 文件

```
rosrun ros-slam pcd_write
```

上条命令在哪个目录执行，就在哪个目录生成 pcd 文件，查看 pcd 点云文件

```
pcl_viewer file.pcd
```

## 软件包搭建过程

### 测试环境

从头开始，不管前面的项目，创建文件夹

```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
```

创建软件包

```
catkin_create_pkg ros-slam rospy roscpp std_msgs urdf xacro gazebo_ros gazebo_ros_control gazebo_plugins
```

进入 ros-slam/src 文件夹创建文件 hello.cpp

```
cd ros-slam/src
touch hello.cpp
```

写入

```cpp
#include <ros/ros.h>

int main(int argc, char const *argv[])
{
    printf("Hello World!\n");
    return 0;
}
```

在 CMakeLists.txt 添加

```bash
echo "add_executable(hello src/hello.cpp)" >> CMakeLists.txt
```

编译

```
cd ~/catkin_ws
catkin_make
```

让 ROS 找到软件包

```bash
source ~/catkin_ws/devel/setup.bash
```

运行节点

```bash
rosrun ros-slam hello
```

运行成功。

### 运行仿真环境

```bash
sudo apt-get install -y ros-noetic-teleop-twist-keyboard
cd ~
git clone -b kinect https://github.com/yym68686/ROS-Lab.git
cd ~/catkin_ws/src
roslaunch ros-slam env.launch
roslaunch ros-slam sensor.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0"
```

CMakeLists.txt 增加

```cmake
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable (pcd_write src/test_pcl.cpp)
add_executable (pcd_write src/pcl_write.cpp)
target_link_libraries(pcd_write
  ${catkin_LIBRARIES}
)
target_link_libraries (pcd_write ${PCL_LIBRARIES})
```

编译

```bash
catkin_make
```



查看当前是否有 camera/depth/ros-slams 话题

```bash
rostopic list
```

运行软件包

```bash

```

点云数据格式：http://docs.ros.org/en/api/sensor_msgs/html/msg/ros-slamCloud2.html

## 
