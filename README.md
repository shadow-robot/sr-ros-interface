[![Build Status](https://api.shippable.com/projects/554b2991edd7f2c052e402be/badge?branchName=indigo-devel)](https://app.shippable.com/projects/554b2991edd7f2c052e402be/builds/latest) [![Circle CI](https://circleci.com/gh/shadow-robot/sr-ros-interface.svg?style=shield)](https://circleci.com/gh/shadow-robot/sr-ros-interface) [![Build Status](https://semaphoreci.com/api/v1/projects/11755ff7-a716-4ac2-a7fb-5edb3c7a87b5/518634/shields_badge.svg)](https://semaphoreci.com/andriy/sr-ros-interface) [![Documentation Status](https://readthedocs.org/projects/shadow-robot/badge)](http://shadow-robot.readthedocs.org/) [![Code Health](https://landscape.io/github/shadow-robot/sr-ros-interface/indigo-devel/landscape.svg?style=flat)](https://landscape.io/github/shadow-robot/sr-ros-interface/indigo-devel) [![codecov.io](http://codecov.io/github/shadow-robot/sr-ros-interface/coverage.svg?branch=indigo-devel)](http://codecov.io/github/shadow-robot/sr-ros-interface?branch=indigo-devel)

# Shadow Robot

[![Shadow Hand manipulates pen](http://img.youtube.com/vi/3WAp_DHwg1c/0.jpg)](http://www.youtube.com/watch?v=3WAp_DHwg1c)

## Overview

This stack focuses on the core software packages built around [our robots](http://www.shadowrobot.com). We'll focus more on the simulated side of things here (as you'll get a full training when receiving one of our robot), but all the code that runs on our simulated robot also runs on the real hardware.

The commands below are mostly summarised in this [cheatsheet](/resources/ShadowROSCheatsheet.pdf)

## Getting Started

### Installation Instructions

We'll assume that you've already installed ROS Indigo from [those instructions](http://wiki.ros.org/indigo/Installation/Ubuntu) using the recommended proposed solution.

To get our software, run the following command from a terminal:
```bash
sudo apt-get install ros-indigo-shadow-robot
```

To get the latest development version, you can also look at those [install steps](INSTALL.md).

### Running a simulated Hand and Arm

#### Running the simulation

To start the hand and arm, run the following command:

```bash
roslaunch sr_hand gazebo_arm_and_hand.launch
```

![Gazebo](/resources/launch_gazebo.png)

*Note: The first time you run gazebo, it might spend some time fetching the model database from the net. If this takes too long the gazebo gui won't start (and you'll get the following error in your terminal:*
`Error [ConnectionManager.cc:116] Failed to connect to master in 30 seconds`*). If this happens, simply kill (pressing ctrl+c) and rerun the previous command.*


#### Running the real hand driver

Although you should have received an already installed machine with the hand delivery, you can find the commands for starting [your system here](http://shadow-robot-ethercat-driver.readthedocs.org).

#### Sending your first commands to the robot

Let's send a command to the proximal joint of the first finger:

```bash
rostopic pub /sh_ffj3_position_controller/command std_msgs/Float64 1.5
```

We're sending a position command to the Shadow Hand (`/sh`) for its first finger's (`ff`) proximal joint (`j3`). The command is in radians (1.5 in our case) and is a double (`std_msgs/Float64`). The finger goes down.
Kill (ctrl+c) the command when you want. It's streaming the command until you kill it.

You can of course control any other joints of the robot in the same way. For example if we wanted to move the arm down, we could send:

```bash
rostopic pub /sa_ss_position_controller/command std_msgs/Float64 0.5
```

All the joints of the hand are prefixed by `/sh_`, while those of the arm are prefixed by `/sa_` (Shadow Hand, Shadow Arm). Here's a diagram to see which joint is which.

![Annotated Hand Diagram](/resources/annotated_hand.png)

#### Receiving your first data from the robot

The data concerning the joints (position, velocity and effort) is published to the `/joint_states` topic. To display it on the command line, you can run:

```bash
rostopic echo /joint_states
```

## Going further

### GUI

A set of GUI plugins are [available](https://github.com/shadow-robot/sr-visualization) to control our robot.

### Programming
Now that you know how to send and receive data to and from the robot, feel free to take things a step further by looking at the *sr_example* package.

## Contacting us

If you encounter a bug in our code, or need a new feature implemented, please use [github issues](https://github.com/shadow-robot/sr-ros-interface/issues?state=open). Otherwise you can also contact us via email: *software@shadowrobot.com*.
