# hero_chassis_controller

## Overview

-
**Keywords:** RoboMaster, ROS, ros_control, hero_chassis_control

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: ZhanyuLiang<br />
Affiliation: none<br />
Maintainer: ZhanyuLiang, 3229466846@qq.com**

The hero_chassis_controller package has been tested under [ROS] Noetic on 20.04. 
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib
- geometry_msgs
- control_msgs
- realtime_tools

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:3229l/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin build

## Usage

Run the simulation and controller with:

	roslaunch hero_chassis_controller run_simulation_and_controller.launch

## Config files

Config file config

* **controllers.yaml**  Params of hero_chassis_controller and joint_state_controller.

## Launch files

* **run_simulation_and_controller.launch:** Hero chassis only simulation and hero chassis controller

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/3229l/hero_chassis_controller/issues)
.

[ROS]: http://www.ros.org
