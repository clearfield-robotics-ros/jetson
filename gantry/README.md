# Gantry Planner & Simulator Package

There's no denying that a gantry will need a planner as well as a simulator.

## Easy Set up

1) Get the path to your arduino-*version*/libraries/ 

2) Make sure the jetson and rosserial package is the newest one 
```
cd ~/catkin_ws/src/jetson/
git pull
cd ~/catkin_ws/src/rosserial/
git pull
```

or clone if you don't have them
```
cd ~/catkin_ws/src/
git clone https://github.com/clearfield-robotics-ros/jetson.git
git clone https://github.com/clearfield-robotics-ros/rosserial.git
```

3) Make sure the workspace is sourced, and make
```
source ~/catkin_ws/devel/setup.bash
catkin_make
```

4) Create the ros_lib/ library folder as part of your arduino libraries
```
rosrun rosserial_arduino make_libraries.py <PATH_TO_arduino>/arduino-*version*/libraries/
```

5) Sanity check that it is in your arduino libraries folder, and your next arduino code should allow including the custom messages to_gantry_msg and gantry_status.

## Example

An example .ino file can be found in the folder customMsgLib/Example/, with the appropriate syntax.
