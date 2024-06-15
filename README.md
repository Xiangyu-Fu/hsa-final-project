# hsa-final-project

## Overall structure explaination

This repository contains most of the information of our implementation of "hsa-bot", a mobile robot as the course project for Humanoid Sensor and Acturator course at ICS, TUM. We have implemented 

1. the hardware interface communicating with the hsa_bot, both controlling the motor and get the reading from the lidar.
2. the controller to issue `cmd_vel` command (angular velocity and linear velocity) to the robot.
3. the simulated version of hsa_bot on gazebo.
4. the overall pipeline of slam based on the reading from lidar and the wheel speed reading.

The main implementation is located at `workspace/src/hsa_bot`, we have a README.md there for the explaination on the package. The `controller` directory contains the Gamepad controller, communicating over network with ros-core to issue `cmd_vel` command. The `workspace/src/rplidar_ros` direcotory is the submodule contains a submodule to read from the lidar sensor.

![IMG_4520](https://github.com/Xiangyu-Fu/hsa-final-project/assets/54738414/321f1113-b33e-4f6b-8a5d-2c724b44eef0)


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

Clone the project, and initialize the submodule as well.
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

