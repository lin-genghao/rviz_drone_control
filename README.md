## rviz_drone_control

这是一个rviz插件，可以在rviz上显示按键来控制PX4无人机


# 环境安装
ubuntu 20.04 ros noetic 

```bash

apt install qtbase5-dev qt5-qmake qtbase5-dev-tools libqt5svg5-dev qtbase5-private-dev libeigen3-dev libgtest-dev libsdl-image1.2-dev libsdl1.2-dev ros-noetic-mavros* build-essential python3-catkin-tools ros-noetic-qt-*

```

# 源码编译

```bash
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
git clone http://code.mmcuav.com:8003/ainav/rviz_drone_control.git
catkin build rviz_drone_control
rospack plugins --attrib=plugin rviz
```

# 界面设计 

## PyQt5 及 pyqt5-tools 安装
```
pip install pyqt5
pip install pyqt5-tools
```
## 查看版本
pip show pyqt5

安装图形界面Qt Designer
sudo apt-get install qttools5-dev-tools
sudo apt-get install qttools5-dev

## 安装之后可执行文件designer在/usr/lib/x86_64-linux-gnu/qt5/bin/下
### 打开 Qt Designer
```
/usr/lib/x86_64-linux-gnu/qt5/bin/designer 
or
designer
```

## 设计界面
在Qt Designer交互界面获得.ui文件

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
