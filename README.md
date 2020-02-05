# turtlesim_controller

## Install

### Environment

- Ubuntu 18.04
- ROS Melodic
  
### Compoling

```bash
sudo apt install ros-melodic-turtlesim
sudo apt install ros-melodic-geometry-msgs
cd ~/catkin_ws/src
git clone https://github.com/RenFukatsu/turtlesim_controller.git
catkin build
source ~/catkin_ws/devel/setup.bash
```

## Run

terminal1

```bash
roscore
```

terminal2

```bash
rosrun turtlesim turtlesim_node
```

terminal3

```bash
roslaunch turtlesim_controller turtlesim_controller
```
