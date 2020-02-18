This is a ROS framework to simulate and control several drones together. It can be used to simulate swarm behavior for quad-rotors using a realistic simulator. The quadrotor simulator is an amended version of the hector_quadrotor framework from TU Darmstadt.

## Install ROS and relevant packages 
### ROS Distro
Install ROS Melodic  (full version)as per: http://wiki.ros.org/melodic/Installation/Ubuntu
Other versions should also work.

### Additional Packages
Hector-quadrotor needs Qt4 and the geographic-msgs structure, so run:
```
sudo apt-get install qt4-default
sudo apt-get install ros-$ROS_DISTRO-geographic-msgs
```

### Download repo
```
git clone git@github.com:coppolam/ros_mav_swarm_simulator.git
```

## Build
```
cd ros_mav_swarm_simulator
catkin_make
source devel/setup.bash
```

## Launch
There are other launch files with other number of MAVs (check mav_controller/launch)

To launch an example with 3 MAVs:
```
roslaunch mav_controller 3mavs.launch
```

In this default example, the MAVs will move around in various triangle patterns.
