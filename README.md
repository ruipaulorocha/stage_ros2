# stage_ros2
Package which contains ROS2 specific hooks and tools for the Stage simulator.

### build
```
cd ~your_ws/src/
git clone https://github.com/rtv/Stage.git # stage lib
git clone https://github.com/n0nzzz/stage_ros2.git # stage_ros2 wrapper
cd ~your_ws/
colcon build
```

### launch
```
cd your_ws/
. install/setup.bash
ros2 run stage_ros stageros src/stage_ros2/world/willow-erratic.world
```
