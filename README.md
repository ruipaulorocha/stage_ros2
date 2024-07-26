# stage_ros2
Package which contains ROS2 specific hooks and tools for the Stage simulator.
This version was forked from this [https://github.com/Navifra-Sally/stage_ros2](https://github.com/Navifra-Sally/stage_ros2) and underwent small modifications by Rui P. Rocha to be used in ROS2 Humble and ROS2 Jazzy distributions.

### Required library that needs to be compiled from source

#### Dependency that needs to be installed in ROS Humble before compilation
```bash
sudo apt-get install libfltk1.1
```

#### Dependencies that need to be installed in in ROS Jazzy before compilation

```bash
sudo apt install libltdl-dev libboost-thread-dev
```

##### Also download and install from .deb files:

- [libfltk1.1_1.1.10-26ubuntu2_amd64.deb](https://drive.google.com/file/d/1NGtQfxx7qYxD7g4FBOBtB1MaXuxNvJqn/view?usp=share_link)

- [libfltk1.1-dev_1.1.10-26ubuntu2_amd64.deb](https://drive.google.com/file/d/1BfMw4me4gWUHk9he_H27dPevdbZjFglx/view?usp=share_link)

```bash
sudo apt install ./libfltk1.1_1.1.10-26ubuntu2_amd64.deb

sudo apt install ./libfltk1.1-dev_1.1.10-26ubuntu2_amd64.deb
```


#### Cloning and compiling the Stage library
```bash
git clone https://github.com/rtv/Stage.git ~/Stage
cd ~/Stage
mkdir build && cd build
cmake ..
make && sudo make install
```
  
### Build ROS2 Stage wrapper
```bash
cd ~/your_ws/src/
git clone https://github.com/ruipaulorocha/stage_ros2.git # stage_ros2 wrapper
cd ~/your_ws/
colcon build
```

### Launch ROS2 Stage wrapper
```bash
cd ~
source .bashrc
```

```bash
# ros2 run stage_ros2 stageros (world file Absolute path)
ros2 run stage_ros2 stageros ~/your_ws/install/stage_ros2/share/stage_ros2/world/willow-erratic.world
```
