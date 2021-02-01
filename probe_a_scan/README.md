# Probe with A-SCAN application

## requirements
It uses the the python_gui package; clone it in a catkin workspace

```bash
git clone https://github.com/gborghesan/python_gui.git
```

In case you use the ubuntu 20.04 and python 3, use the branch noetic

Optional, a node to visualize the poses in the json file is in

[https://gitlab.kuleuven.be/ras/RAS-Projects/lwr_ultra_sound/pose_probe_visualization.git](https://gitlab.kuleuven.be/ras/RAS-Projects/lwr_ultra_sound/pose_probe_visualization.git)


## start-up

1. in a shell, launch the lanchfile that brings up the event sender and RVIZ

```bash
roslaunch  probe_a_scan load_setup_lwr_a_mode.launch
```

2. in a shell in the script/deploy folder launch the orocos process

```bash
rttlua -i my_deploy.lua
```

## usage

the first 4 buttons of the gui starts and stop the automatic probing position mode and the impdance mode

The other buttons save the current position in a file to be used in probing mode. _To be used only in Admittance Mode_

## TODO

- check that the force guard works. the force sensor is not integrated in the deployment script
- there is a state for saving the a-scan, that stops for 1 second. this state is currently only printing on screen. some function for activating and deactivating the a-scan should be implemented here.


