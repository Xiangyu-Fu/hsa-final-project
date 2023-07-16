# HSA_BOT

**Lecture: Humanoid Sensor and Actuator**

SLAM mapping and navigation simulation project

 In this SLAM Mapping and Navigation Simulation Project, there are three main parts, which are
 - Construction of models and environments
 - Location and navigation based on known maps
 - Map building and navigation using gmapping

## Start up
```bash
$ sudo apt-get install ros-noetic-move-base-msgs
$ sudo apt-get install ros-noetic-navigation
$ sudo apt-get install ros-noetic-map-server
$ sudo apt-get install ros-noetic-move-base
$ rospack profile
$ sudo apt-get install ros-noetic-amcl
$ sudo apt-get install ros-noetic-slam-gmapping

```

## How to Launch
```bash
$ roslaunch hsa_bot sim.launch
```

## How to Create a map

### simulator
Open a new terminal, open `rqt`, and open robot steering plugin.
```bash
rosrun gmapping slam_gmapping scan:=/hsa_bot/laser/scan
```

Then save the map
```bash
rosrun map_server map_saver
```

## real world

Frist, change the `frame_id` to `base_link` in `rplidar_a1.launch`.


```bash
$ roslaunch hsa_bot real.launch
```