# Source Install
To get the latest version of our software, you can install it from source. We're doing our best to keep the indigo-devel branch stable.

You'll first need to [install ROS](http://wiki.ros.org/indigo/Installation/Ubuntu).

## Workspaces
As explained in this [ROS tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), to install from source you first need to create a workspace.

At Shadow, we recommend overlaying different workspaces. For example, if you develop some code that uses our software, you should probably have a `development` workspace containing your code based on top of a `shadow` workspace that contains ours. The idea is that you don't need to recompile all our code (or update it) when you're developing your own code.

### Creating this overlay hierarchy
We'll explain how to create the 2 workspaces as explained above.

Let's create the `shadow` overlay. It will overlay the standard ROS packages (installed via `apt-get`).

  - first make sure you're using the base install:

```bash
source /opt/ros/indigo/setup.bash
```

  - now let's create the folders you'll need for this first overlay

```bash
mkdir -p ~/workspace/shadow/src
```

  - and initialise the workspace

```bash
cd ~/workspace/shadow/src
catkin_init_workspace
cd ~/workspace/shadow
catkin_make
```

  - your first workspace is created, you now need to load it before creating the second workspace that will overlay it.

```bash
source ~/workspace/shadow/devel/setup.bash
```

Let's create the second overlay: `development` which will contain your own code. You'll repeat the steps above, except that you're using the setup.bash from the `shadow overlay`.

   - first make sure you're using the base install:

```bash
source ~/workspace/shadow/devel/setup.bash
```

   - now let's create the folders you'll need for this second overlay

```bash
mkdir -p ~/workspace/development/src
```

   - and initialise the workspace

```bash
cd ~/workspace/development/src
catkin_init_workspace
cd ~/workspace/development
catkin_make
```

   - your second workspace is created, and it is overlaying the `shadow` workspace. You need to load it (each time you open a new terminal).

```bash
source ~/workspace/development/devel/setup.bash
```

To load this workspace automatically, it is a very good idea to add it to your `~/.bashrc`:

```bash
echo "source ~/workspace/development/devel/setup.bash" >> ~/.bashrc
```

## Populating the Shadow workspace with the code
To get the source code, we use [wstool](http://wiki.ros.org/wstool).

 - install it:

```bash
sudo apt-get install python-wstool
```

 - initialise the `wstool` workspace

```bash
cd ~/workspace/shadow/src
wstool init
```

 - use our rosinstall file to install the different stacks

```bash
wstool merge https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/data/shadow_robot-indigo.rosinstall
wstool update
```

 - install the dependencies

```bash
rosdep install --from-paths ~/workspace --ignore-src --rosdistro=indigo
```

 - compile the code

```bash
cd ~/workspace/shadow
catkin_make
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
