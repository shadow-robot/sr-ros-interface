Hand Configuration
==================

-  `Calibrations <calibrations>`__ - Both Left and Right hand
   calibration files, including the calibration of each joint and the 2
   pressure sensors per joint. These are set by Shadow Robot prior to
   the delivery of a hand and should only be changed by a person that
   has received training for the calibration procedure.
-  `Controls <controls>`__ - Yaml files containing definitions for
   calibration, effort, joint, position and motor controllers
-  `Demos <demos>`__ - Demo files for store position (demo\_rs.py or
   demo\_ls.py), general demo (demo\_r.py or demo\_l.py), and card trick
   demo (demo\_rc.py).
-  `Launch <launch>`__ - Launch files for Left/Right/Bimanual hands,
   specific to an install
-  `Mappings <mappings>`__ - The file which provides the mapping between
   hand joints and actuators is specified here.
-  `Rates <rates>`__ - Sensor and actuator update rate files.

Joint to actuator mappings
--------------------------

The mapping files for each type of hand can be found
`here <https://github.com/shadow-robot/sr-ros-interface-ethercat/tree/indigo-devel/sr_edc_launch/mappings/default_mappings>`__.
The mapping which corresponds to the system that you are launching, i.e.
right hand/left hand/motor actuated/muscle actuated, should be specified
in the corresponding lh (left hand) or rh (right hand)
load\_joint\_mapping.xml, in the mappings directory.

Changing the joint to actuator mapping should be done with caution, as
an incorrect mapping could cause damage to the hand.

Launching a hand
----------------

This directory includes launch files for left hand/right hand/bimanual
systems.

The ethernet port in which the hand is connected is specified in the
launch file. If the port is changed, then it should be edited here. The
hand\_serial parameter is hand specific, so should only be changed if
launching a different hand. To find the hand serial you can launch the
hand without the hand\_serial argument and then check the program
output. You should see something like:

.. code:: bash

    Trying to read mapping for: /hand/mapping/1178

To launch a right hand, run:

.. code:: bash

    roslaunch sr_ethercat_hand_config sr_rhand.launch

To launch a left hand:

.. code:: bash

    roslaunch sr_ethercat_hand_config sr_lhand.launch

And for a bimanual system:

.. code:: bash

    roslaunch sr_ethercat_hand_config sr_system.launch

Running a demo
--------------

The demo scripts can be run via rosrun, with a command of the following
format:

For the right hand:

.. code:: bash

    rosrun sr_ethercat_hand_config demo_r.py

And for the left hand:

.. code:: bash

    rosrun sr_ethercat_hand_config demo_l.py

