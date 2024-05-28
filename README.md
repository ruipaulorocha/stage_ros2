# stage_ros2
Package which contains ROS2 specific hooks and tools for the Stage simulator.
This version was forked from this [https://github.com/Navifra-Sally/stage_ros2](https://github.com/Navifra-Sally/stage_ros2) and underwent small modifications by Rui P. Rocha to be used in ROS2 Humble distro.
  
### Requirement
```bash
git clone https://github.com/rtv/Stage.git ~/Stage# stage lib
cd ~/Stage
mkdir build && cd build
cmake ..
make && sudo make install
```
  
### build
```bash
cd ~/your_ws/src/
git clone https://github.com/ruipaulorocha/stage_ros2.git # stage_ros2 wrapper
cd ~/your_ws/
colcon build
```

### launch
```bash
cd ~
source .bashrc
```

```bash
# ros2 run stage_ros2 stageros (world file Absolute path)
ros2 run stage_ros2 stageros ~/your_ws/install/stage_ros2/share/stage_ros2/world/willow-erratic.world
```
