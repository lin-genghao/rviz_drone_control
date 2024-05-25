## rviz_drone_control

这是一个rviz插件，可以在rviz上显示按键来控制PX4无人机

# 源码编译

```bash
mkdir ~/catkin_ws/src
cd ~/catkin_ws/src
git clone http://code.mmcuav.com:8003/ainav/rviz_drone_control.git
catkin build rviz_drone_control
```

# 运行rviz_drone_control

```
source ~/catkin_ws/devel/setup.bash
roslaunch rviz_drone_control rviz_drone_control.launch
```

# 效果展示

![Video Title](attachment/多机模式.mp4)

# 依赖

rviz_drone_control 在 Ubuntu20.04上测试，需要依赖

1. ros noetic
2. mavros
3. rviz
