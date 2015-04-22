# Shadow Robot

## Overview

This stack focuses on the core software packages built around [our robots](http://www.shadowrobot.com). We'll focus more on the simulated side of things here (as you'll get a full training when receiving one of our robot), but all the code that runs on our simulated robot also runs on the real hardware.

The commands below are mostly summarised in this [cheatsheet](/docs/resources/ShadowROSCheatsheet.pdf)

## Getting Started

### Installation Instructions

TODO

### Running a simulated Hand and Arm

#### Running the simulation

To start the hand and arm, run the following command:

```bash
roslaunch sr_hand gazebo_arm_and_hand.launch
```



*Note: The first time you run gazebo, it might spend some time fetching the model database from the net. If this takes too long the gazebo gui won't start (and you'll get the following error in your terminal:*
`Error [ConnectionManager.cc:116] Failed to connect to master in 30 seconds`*). If this happens, simply kill (pressing ctrl+c) and rerun the previous command.*

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

All the joints of the hand are prefixed by `/sh_`, while those of the arm are prefixed by `/sa` (Shadow Hand, Shadow Arm). Here's a diagram to see which joint is which.


#### Receiving your first data from the robot

The data concerning the joints (position, velocity and effort) is published to the '''/joint_states''' topic. To display it on the command line, you can run:

```bash
rostopic echo /joint_states
```

## Going further

### GUI

A set of GUI plugins are [sr_visualization](available) to control our robot.

### Programming

Now that you know how to send and receive data to and from the robot, we can go one step further and write a simple script that [[sr_hand/Tutorials/Sending Commands [python]|sends a target to a joint]], then something a bit more useful that reads the position from one finger joint and sets it as the target for
another finger joint [[sr_hand/Tutorials/Link two joints [python]|in python]]/[[sr_hand/Tutorials/Link two joints [cpp]|in c++]].

## Contacting us

If you encounter a bug in our code, or need a new feature implemented, please use [https://github.com/shadow-robot/sr-ros-interface/issues?state=open)[github issues]. Otherwise you can also contact us via email: *software@shadowrobot.com*.
