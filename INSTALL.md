# Source Install
To get the latest version of our software, you can install it from source. We're doing our best to keep the indigo-devel branch stable.

You'll first need to [install ROS](http://wiki.ros.org/indigo/Installation/Ubuntu).

## Workspaces

We created a one liner script that installs everything for you:
 - ros and dependencies
 - creates the proper workspaces hierarchy (we are pulling a few dependencies from source)
 - compiles everything

You simply need to run (replace `-w ~/projects/shadow_robot/base` with the path you want to install the sources in):

```bash
curl -L bit.ly/dev-install | bash -s -- -w ~/projects/shadow_robot/base
```

## Installing for a real robot

If you're installing the code for a real robot, you'll first need to follow the above instructions in order to have the workspace installed from source. Then you will need to pull the `sr-config` branch corresponding to your robot. `sr-config` contains the parameters specific to your hand (calibration, controller tuning etc...).

You can check which branch is installed on the computer provided by Shadow by running:

```bash
roscd sr_ethercat_hand_config
git branch
```

The highlighted branch is the one that is currently used. Let's assume it's `shadowrobot_1234` for the following instructions. On the newly installed computer you will need to pull the same branch:

```bash
roscd sr_ethercat_hand_config
git pull
git fetch shadowrobot_12345
```

*Note: the etherCAT configuration has evolved quite a bit in the latest years. If the config is not working for you, get in touch and we'll help you migrate the configuration.*
