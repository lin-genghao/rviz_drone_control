## rviz_drone_control

这是一个rviz插件，可以在rviz上显示按键来控制PX4无人机


# 环境安装
ubuntu 20.04 ros noetic 

```bash

apt install qtbase5-dev qt5-qmake qtbase5-dev-tools libqt5svg5-dev qtbase5-private-dev libeigen3-dev libgtest-dev libsdl-image1.2-dev libsdl1.2-dev ros-noetic-mavros* build-essential python3-catkin-tools ros-noetic-qt-*

sudo apt-get install qtcreator
sudo apt-get install qt4-default
sudo apt-get install libfontconfig1 
sudo apt-get install mesa-common-dev


```

# 源码编译

```bash
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
git clone http://code.mmcuav.com:8003/ainav/rviz_drone_control.git
catkin build rviz_drone_control
rospack plugins --attrib=plugin rviz
```

# 运行rviz_drone_control

```
source ~/catkin_ws/devel/setup.bash
roslaunch rviz_drone_control rviz_drone_control.launch
```

# 效果展示

![多机模式](attachment/多机模式.mp4)

# 依赖

rviz_drone_control 在 Ubuntu20.04上测试，需要依赖

1. ros noetic
2. mavros
3. rviz
