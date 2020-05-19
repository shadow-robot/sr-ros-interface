sr\_utilities
=============

This package contains utility libraries. There are examples of using
python implementation of HandFinder utility in
`HandCommander <../sr_robot_commander/doc/tutorial/HandCommander.md>`__
and
`RobotCommander <../sr_robot_commander/doc/tutorial/RobotCommander.md>`__.
Check the `test file <test/test_hand_finder.cpp>`__ for an example of
using C++ implementation of HandFinder.

ROS interface
-------------

Controller Spawner
******************

**controller_spawner.py** spawns the ROS controllers necessary for Shadow dexterous hands, based on a config file.

The default config file is in `sr_interface/sr_robot_launch/config/controller_spawner.yaml <https://github.com/shadow-robot/sr_interface/blob/melodic-devel/sr_robot_launch/config/controller_spawner.yaml>`__. The config file contains all possible ROS controllers, organised into controller groups. You can specify a different config file using the `config_file_path` parameter.

When the controller spawner is run, it finds any running Shadow dexterous hands, and based on the `controller_group` parameter, launches the subset of controllers appropriate for the joints in the running hands. It also puts any relevant controller parameters defined in the `controller_configs` section of the config file on the parameter server.

Other controller groups defined in the config file are also stored on the ROS parameter server under `/controller_groups` for ease of controller group switching later.

Any controllers defined in the config file, relevant to the currently running hands, but not part of the requested controller group will be stopped.

Wrist joints can be excluded from the launched controllers using the `exclude_wrist` parameter.

How long the spawner waits for controller management services to come up is defined by the `service_timeout` parameter.

The `wait_for` parameter can be used to specify the name of a topic that the spawner will wait for before attempting to spawn controllers.

*Examples:*

For trajectory and position controllers

.. code:: bash

    rosrun sr_utilities controller_spawner.py _controller_group:=trajectory

Spawns only the position controllers  

.. code:: bash

    rosrun sr_utilities controller_spawner.py _controller_group:=position

In some cases it may be better to control the wrist joints with the arm trajectory. 
To remove the joints from the hand trajectory set a private parameter *~exclude_wrist* to true.

.. code:: bash

    rosrun sr_utilities controller_spawner.py _exclude_wrist:=true
