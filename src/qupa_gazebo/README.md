# Qupas Package

The qupa robot is a robotic project base on swarm behaviour. Developed in Ecuador for research purposes this is a simulation package to test different behaviours on the qupa robot using ROS Noetic and Gazebo Simulator.

# Prerequisites

1. [ROS1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
2. [Gazebo 11 or Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=quick_start)

# Content

1. [Build](#1build)
2. [Run](#2run)
3. [Package Layout](#3package-layout)
## 1.Build 

If you are not familiarized with ROS environment I recommend to check the turtlebot3 package to get used to the framework. That said, enjoy using the simulator.
### Build package
To build the package you will have to clone the repository on a local directory. Follow the steps or check the video below:

```bash
mkdir -p ~/qupa_ws/src
cd ~/qupa_ws/src
# I have not uploaded the repository yet
git clone "https://github.com/davatorr1009/qupa_gazebo.git"
cd ..
catkin_make
```
Here's a video to help on the guidance:

**Video**



## 2.Run
We can run the simulation using the launch file in the qupa_simulation package

### Launch files

Inside this directory we'll have the following files

- `multi_qupa.launch` : This launch file is related to launching a Gazebo World with some detailed arguments such as:
    
    |`Argument` | Description |
    |--------------|----------|
    | `world` | Set the desired world file |
    | `use_ir` | Add infrared sensors to the simulation |
    | `high_res` | Add camera sensors instead of logical cameras |
    | `qupa_1` | Robot's namespace for the first robot |
    | `qupa_2` | Robot's namespace for the second robot |
    | `use_rviz` | Visualize rviz|
    | `first_qp_x_pos`| First qupa x axis spawn position|
    | `first_qp_y_pos`| First qupa y axis spawn position|
    | `first_qp_z_pos`| First qupa z axis spawn position|
    | `first_qp_z_pos`| First qupa yaw spawn orientation|

---

Here's an example on how to run the file

```bash
cd ~/qupa_ws
source devel/setup.bash
roslaunch qupa_simulation multi_qupa.launch 
```

It should open the default world in Gazebo as shown here:



### Exploration test node

To run some test behaviour on the diff drive It was developed a basic node that moves the robot around in random orientations. To run this node we can use the following.

```bash
cd ~/qupa_ws/src/qupa_behaviour/scripts
chmod +x random_exploration.py
cd ~/qupa_ws
source devel/setup.bash
# Running example if you want the qupa_1 to do the exploration
rosrun qupa_behaviour random_exploration.py qp_1
```

## 3.Package Layout

### Qupa

- `qupa`
    - `qupa_description:` URDF, mesh files describing the robot and launch
    - `qupa_simulation:`     launch, models, worlds 
    - `qupa_msgs:`     Custom qupa msgs
    - `qupa_behaviour:`     qupa behaviour nodes
