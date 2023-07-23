# hsa-final-project

## ROS requirement

version: ROS noetic

### required packages

Install the packages first:
```bash
$ sudo apt-get install ros-noetic-move-base-msgs
$ sudo apt-get install ros-noetic-navigation
$ sudo apt-get install ros-noetic-map-server
$ sudo apt-get install ros-noetic-move-base
$ sudo apt-get install ros-noetic-amcl
$ sudo apt-get install ros-noetic-slam-gmapping
```

### How to run ROS simulation testing 

Clone the project
```bash
$ git clone git@github.com:Xiangyu-Fu/hsa-final-project.git
$ git submodule update --init --recursive
```

Build the workspace, notice the package `rplidar_ros` only required in real robot.
```bash
$ cd workspace
$ catkin build
$ source devel/setup.bash
```

Run the simulation,
```bash
$ roslaunch hsa_bot sim.launch
```

Now, only Rviz should be opened, and it look like this:

![image](https://github.com/Xiangyu-Fu/hsa-final-project/assets/54738414/c326f33c-69ce-473b-b201-3fcb60e71a58)



## Real Rboot System requirement

Machine: Raspberry Pi 4B

OS: Ubuntu Server 20.04 LTS

## Python requirement

- `evdev` for linux input system read

