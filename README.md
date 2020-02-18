# Install 

## ROS Distro
Install ROS Melodic  (full version)as per: http://wiki.ros.org/melodic/Installation/Ubuntu
Other versions should also work.

## Additional Packages
Hector-quadrotor needs Qt4 and the geographic-msgs structure, so run:
```
sudo apt-get install qt4-default
sudo apt-get install ros-$ROS_DISTRO-geographic-msgs
```

## Download repo
```
git clone ros_swarm_simulator
```

# Build
## Build
```
cd ros_swarm_simulator
catkin_make
```

# Launch
```
roslaunch mav_controller 3mavs.launch
```