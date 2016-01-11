Source Install
==============

To get the latest version of our software, you can install it from
source. We're doing our best to keep the indigo-devel branch stable.

Workspaces
----------

We created a one liner script that installs everything for you: - ros
and dependencies - creates the proper workspaces hierarchy (we are
pulling a few dependencies from source) - compiles everything

You simply need to run (replace
``-w ~{{ros_user}}/projects/shadow_robot/base`` with the path you want
to install the sources in, make sure you keep the ``~{{ros_user}}`` if
you want the project in your home folder or that you point to a
directory with write permission for your user):

.. code:: bash

    curl -L bit.ly/dev-machine | bash -s -- -w ~{{ros_user}}/projects/shadow_robot/base

The output is quite verbose:

.. code:: bash

      % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                     Dload  Upload   Total   Spent    Left  Speed
    100   177  100   177    0     0    465      0 --:--:-- --:--:-- --:--:--   465
    100  4146  100  4146    0     0   6281      0 --:--:-- --:--:-- --:--:--  6281
    =================================================================
    |       Installing Shadow Robot development environment

[...]

.. code:: bash

    -------------------
     | Running Ansible |
     -------------------

    sudo password: 


    PLAY [ros-hydro-desktop-precise64] ******************************************** 
    skipping: no hosts match

[...]

.. code:: bash

    PLAY RECAP ******************************************************************** 
    changed: [localhost] => {"changed": true, "cmd": "usermod -a -G dialout 'ugo' ", "delta": "0:00:00.009358", "end": "2016-01-04 08:33:52.390686", "item": "", "rc": 0, "start": "2016-01-04 08:33:52.381328", "stderr": "", "stdout": ""}
    localhost                  : ok=38   changed=26   unreachable=0    failed=0   


     ------------------------------------------------
     | Install complete, please restart the machine |
     ------------------------------------------------

Updating your workspace
-----------------------

For a quick update of the main workspace, we added an alias: simply run
``ws_update`` from the console to get the latest source code.

To fully update the different workspaces (both the shadow code and the
dependencies), you can rerun the install script. **WARNING: If you use
the same workspace path, it'll completely overwrite it**

Installing for a real robot
---------------------------

If you're installing the code for a real robot, you simply need to add
an option to the line above to pull the proper ``sr-config`` branch.
``sr-config`` contains the parameters specific to your hand
(calibration, controller tuning etc...).

You can check which branch is installed on the computer provided by
Shadow by running (on the machine provided with your hand):

.. code:: bash

    roscd sr_ethercat_hand_config
    git branch

The highlighted branch is the one that is currently used. Let's assume
it's ``shadowrobot_1234`` for the following instructions. On the newly
installed computer you will need to pull the same branch:

.. code:: bash

    curl -L bit.ly/dev-machine | bash -s -- -w ~/projects/shadow_robot/base -c shadowrobot_1234

*Note: the etherCAT configuration has evolved quite a bit in the latest
years. If the config is not working for you, get in touch and we'll help
you migrate the configuration.*
