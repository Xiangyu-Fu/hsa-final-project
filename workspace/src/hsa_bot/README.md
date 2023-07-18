# HSA_BOT

**Lecture: Humanoid Sensor and Actuator**

SLAM mapping and navigation simulation project

 In this SLAM Mapping and Navigation Simulation Project, there are three main parts, which are
 - Construction of models and environments
 - Location and navigation based on known maps
 - Map building and navigation using gmapping

## 1. Start up
Here you need to install all the packages that need in our env.
```bash
$ sudo apt-get install ros-noetic-move-base-msgs
$ sudo apt-get install ros-noetic-navigation
$ sudo apt-get install ros-noetic-map-server
$ sudo apt-get install ros-noetic-move-base
$ rospack profile
$ sudo apt-get install ros-noetic-amcl
$ sudo apt-get install ros-noetic-slam-gmapping

```

## 2. How to Launch in Gazebo
```bash
$ roslaunch hsa_bot sim.launch
```

### How to Create a map
Open a new terminal, open `rqt`, and open robot steering plugin.
```bash
rosrun gmapping slam_gmapping scan:=/hsa_bot/laser/scan
```

Then save the map
```bash
rosrun map_server map_saver
```

## 3. How to Launch in real robot

Some preparations:
1. Change the `frame_id` to `base_link` in `rplidar_a1.launch`.
2. insert the following command in your `/.bashrc`
```
alias hsa_mode='export ROS_IP=192.168.178.22 && export ROS_MASTER_URI=http://ubuntu:11311'
```


In real robot you need to run:
```bash
$ roslaunch hsa_bot real.launch has_map:=false
```

In your local computer:
```
hsa_mode
rviz
```


(May be required)For remote control:
```bash
$ sudo apt-get install ros-noetic-key-teleop
$ rosrun key_teleop key_teleop.py
```

## 4. Additional requirement

1. Since we have both `ttyUSB0` and `ttyUSB1` in the connection, we assume 1) `ttyUSB00` lidar and 2) `ttyUSB1` as the serial port. So you need to ensure that lidar is plugged-in before serial port.


