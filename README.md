# rse_dam

## Summary

This repository contains the software used in my MS Thesis Project, "Preliminary Implementation of a Modular Control System for Dual-Arm Manipulation with a Humanoid Robot", which involved creating a baseline control architecture based on the [Resilient Spacecraft Executive (RSE) Architecture](https://arc.aiaa.org/doi/10.2514/6.2016-5541) from MIT, WHOI, NASA JPL, and Caltech. 

For inital tests I used the ["Boxbot"](https://github.com/MatthewVerbryke/boxbot_ros) robot. The robot is controlled by two separate computers (one for each arm), which comunicate over ethernet using rosbridge.

See [paper](https://etd.ohiolink.edu/pg_10?::NO:10:P10_ETD_SUBID:174789) for a more detailed breakdown of the project and software.

## Recommended OS/Programs

The software was developed and tested in:
 - Ubuntu 16.04 LTS
 - ROS Kinetic
 - Gazebo 7.13
 - MoveIt! for ROS Kinetic

## Installation

This program has several dependencies:

 - [arbotix_ros](https://github.com/MatthewVerbryke/arbotix_ros)
 - [boxbot_ros](https://github.com/MatthewVerbryke/boxbot_ros)
 - [rss_git_lite](https://github.com/cmcghan/rss_git_lite)
 - [widowx_arm](https://github.com/MatthewVerbryke/widowx_arm)
 
Install these dependencies, then install this program using git clone into the catkin workspace of your choice (probably a good Idea to use a fresh one).

## Usage

The program is launched using the ```multilaunch.py``` program on each computer. the arguments to this file are as follows:
```
multilaunch.py $COMPUTERNAME$ $SIMULATED$ $WORLDFILE$ $HLCALL$ $DLCALL$
```

where:

 - ```$COMPUTERNAME$``` is the name of your computer
 - ```$SIMULATED$``` and ```$WORLDFILE$``` tells the system if it is running in RViz only (```$SIMULATED$``` = "true"), on the actual hardware (```$WORLDFILE$``` = "real") a Gazebo simulation (neither of the above with a world file name).
 - ```$HLCALL$``` is the Habitual Layer script that should be run
 - ```$DLCALL$``` is the Deliberative Layer script that should be run (only one computer in the system should run it)
 
Whichever computer runs the deliberative script (the "primary" computer) should be launched about a second or so before the other, so that the rosbridge servers have a chance to launch before programs begin to start piping over them.

## Future Work

The system as-is is very far from feature complete, and is currently limited in what robots it works on and what it can do. Several lines of work are being persued inorder to improve the system. See the paper for more details.

## License
 
Unless otherwise noted in a file, This program is licensed under the BSD 3-clause license, as presented in the LICENSE file

Program is Copyright (c) University of Cincinnati

