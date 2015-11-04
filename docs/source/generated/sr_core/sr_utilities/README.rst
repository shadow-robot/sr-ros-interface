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

| **trajectory\_controller\_spawner.py** checks the hands installed on
the system (with HandFinder library) and finds the actual hand joints by
parsing the URDF from parameter server (robot\_description). It the
position controllers for joints are not spawned the node spawn them.
Afterwards it checks the parameter server for *~hand\_trajectory*
parameter and if it exists and its value is true it will spawn the
trajectory controller for the hands.
| If only the position controllers are required, node can be run without
the *~hand\_trajectory* parameter or setting its value to false.
| *Examples:*
| For trajectory and position controllers

.. code:: bash

    rosrun sr_utilities trajectory_controller_spawner.py _hand_trajectory:=true

Spawns only the position controllers

.. code:: bash

    rosrun sr_utilities trajectory_controller_spawner.py _hand_trajectory:=false

In some cases it may be better to control the wrist joints with the arm
trajectory. To remove the joints from the hand trajectory set a private
parameter *~exclude\_wrist* to true. \`\`\`bash rosrun sr\_utilities
trajectory\_controller\_spawner.py *exclude*\ wrist:=true
