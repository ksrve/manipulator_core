# Manipulator
## Autonomous Chemical Laboratory

This is a project aimed at developing autonomous robotic laboratory assistants for chemical experiments

## Features

- Control system for robotic manipulators
- Communication with the website


## Tech

Manipulator uses a number of open source projects to work properly:

- [ROS](https://www.ros.org/) - a set of software libraries and tools for robotic applications
- [Moveo BCN3D](https://github.com/BCN3D/BCN3D-Moveo) - Manipulators construction

And of course Manipulator itself is open source with a [public repository][dill]
 on GitHub.

## Installation

> Make sure you have ROS installed correctly with a functioning workspace.
> We used ROS Noetic on Ubuntu 20.04 (if you have a different distro, you may need to change some things).

Install the repository and start the experiment!

```sh
cd ~catkin_ws/src
git clone https://github.com/ksrve/manipulator_core
cd ..
catkin build
```

Establish rosserial node that communicates with Arduino
```sh
rosrun rosserial_python serial_node.py /dev/ttyUSB0 
```

Start control with this command!
```sh
rosrun manipulator_core manipulator_1.py 
```


## Troubleshooting
If you get the following __error__: 
manipulator_core/ArmJointANgle.h: No such file or directory", perform the following steps in terminal:

```sh
 cd <Arduino sketchbook>/libraries
 rm -rf ros_lib 
 rosrun rosserial_arduino make_libraries.py .
 ```
