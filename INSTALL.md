# Source Install
To get the latest version of our software, you can install it from source. We're doing our best to keep the indigo-devel branch stable.

## Workspaces

We created a one liner script that installs everything for you:
 - ros and dependencies
 - creates the proper workspaces hierarchy (we are pulling a few dependencies from source)
 - compiles everything

You simply need to run (replace `-w ~{{ros_user}}/projects/shadow_robot/base` with the path you want to install the sources in, make sure you keep the `~{{ros_user}}` if you want the project in your home folder or that you point to a directory with write permission for your user):

```bash
curl -L bit.ly/dev-machine | bash -s -- -w ~{{ros_user}}/projects/shadow_robot/base
```

The output is quite verbose:

```bash
  % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                 Dload  Upload   Total   Spent    Left  Speed
100   177  100   177    0     0    465      0 --:--:-- --:--:-- --:--:--   465
100  4146  100  4146    0     0   6281      0 --:--:-- --:--:-- --:--:--  6281
=================================================================
|       Installing Shadow Robot development environment
```
[...]
```bash
-------------------
 | Running Ansible |
 -------------------

sudo password: 


PLAY [ros-hydro-desktop-precise64] ******************************************** 
skipping: no hosts match
```
[...]
```bash
PLAY RECAP ******************************************************************** 
changed: [localhost] => {"changed": true, "cmd": "usermod -a -G dialout 'ugo' ", "delta": "0:00:00.009358", "end": "2016-01-04 08:33:52.390686", "item": "", "rc": 0, "start": "2016-01-04 08:33:52.381328", "stderr": "", "stdout": ""}
localhost                  : ok=38   changed=26   unreachable=0    failed=0   


 ------------------------------------------------
 | Install complete, please restart the machine |
 ------------------------------------------------
```

## Updating your workspace

For a quick update of the main workspace, we added an alias: simply run `ws_update` from the console to get the latest source code.

To fully update the different workspaces (both the shadow code and the dependencies), you can rerun the install script. **WARNING: If you use the same workspace path, it'll completely overwrite it**

## Installing for a real robot

### Configuration

If you're installing the code for a real robot, you simply need to add an option to the line above to pull the proper `sr-config` branch. `sr-config` contains the parameters specific to your hand (calibration, controller tuning etc...).

You can check which branch is installed on the computer provided by Shadow by running (on the machine provided with your hand):

```bash
roscd sr_ethercat_hand_config
git branch
```

The highlighted branch is the one that is currently used. Let's assume it's `shadowrobot_1234` for the following instructions.

### ROS Indigo

On the newly installed computer with Ubuntu Trusty you will need to pull the same configuration branch:

```bash
curl -L bit.ly/dev-machine | bash -s -- -w ~/projects/shadow_robot/base -c shadowrobot_1234
```
### ROS Kinetic

Please install Ubuntu Xenial and use the following command for ROS Kinetic:
```bash
bash <(curl -Ls https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/ansible/deploy.sh) -r sr-build-tools -b master -i data/shadow_robot-kinetic.rosinstall -v kinetic -t mongodb,pyassimp -с shadowrobot_1234
```

After successfull command execution please run:
```bash
echo 'source $HOME/workspace/shadow_robot-kinetic/base/devel/setup.bash' >> ~/.bashrc 
```

### Notice 

*Note: the etherCAT configuration has evolved quite a bit in the latest years. If the config is not working for you, get in touch and we'll help you migrate the configuration.*

## Installing for a real robot using Docker container

### Docker installation

Your machine should have Ubuntu Trusty or Xenial installed.  
Please install Docker using the following [instructions](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/).

### ROS Indigo Docker container

Pull ROS Indigo container
```bash
docker pull shadowrobot/dexterous-hand:indigo
```

Container created via
```bash
docker run -it --privileged --name hand_e_indigo_real_hw --network=host -e DISPLAY -e QT_X11_NO_MITSHM=1 -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw shadowrobot/dexterous-hand:indigo
```
(you don’t need to run “docker run” every time, as the container is persistent)

To start the container again please execute
```bash
docker start hand_e_indigo_real_hw
```

### ROS Kinetic Docker container

Pull ROS Kinetic container
```bash
docker pull shadowrobot/dexterous-hand:kinetic
```

Container created via
```bash
docker run -it --privileged --name hand_e_kinetic_real_hw --network=host -e DISPLAY -e QT_X11_NO_MITSHM=1 -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw shadowrobot/dexterous-hand:kinetic
```
(you don’t need to run “docker run” every time, as the container is persistent)

To start the container again please execute
```bash
docker start hand_e_kinetic_real_hw
```

### Configuration

If you're installing the code for a real robot, you simply need to pull the proper `sr-config` branch. `sr-config` contains the parameters specific to your hand (calibration, controller tuning etc...).

You can check which branch is installed on the computer provided by Shadow by running (on the machine provided with your hand):

```bash
roscd sr_ethercat_hand_config
git branch
```

The highlighted branch is the one that is currently used. Let's assume it's `shadowrobot_1234` for the following instructions.

In Docker container's console window type the following commands
```bash
roscd sr_ethercat_hand_config
git fetch
git checkout shadowrobot_1234
```

Now you are ready to use Docker container with your hand.

### Launch

Launch the right hand in PWM mode (safe in case of uncalibrated hand or untested sensors)
```bash
roslaunch sr_ethercat_hand_config sr_rhand.launch eth_port:=<Hand's Ethernet port>
```
For the left hand please use
```bash
roslaunch sr_ethercat_hand_config sr_lhand.launch eth_port:=<Hand's Ethernet port>
```

To close the container use CTRL-d or
```bash
exit
```
