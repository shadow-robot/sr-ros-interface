Shadow Robot CyberGlove Configuration
=====================================

Contains the configuration for combining the Shadow Robot Hand with
CyberGlove (per person and per CyberGlove)

-  `Calibrations <calibrations>`__ - Calibration files for CyberGlove to
   Shadow Robot hand
-  `Mappings <mappings>`__ - Files specifying the mapping of sensor
   readings from the CyberGlove to Shadow Robot hand joints.

Adjusting CyberGlove calibration
--------------------------------

Depending on the wearer of the CyberGlove, you may find that the
calibration needs to be adjusted. To do this, identify the name of the
joint that requires adjusting, then find a suitable range of raw
readings from the CyberGlove that match your desired range of motion.
This can be done by monitoring the topic for CyberGlove with the
following command:

.. code:: bash

    rostopic echo raw/joint_states

And selecting an appropriate minimum and maximum value. The file
cyberglove.cal should then be edited, inserting the new raw values taken
from the joint\_states topic, to coincide with the calibrated minimum
and maximum values of that joint, in degrees.

Branch naming convention
------------------------

There are 2 CyberGloves. Glove A has a large control box. Glove B has a
small control box.

For more information on CyberGloves, see `this Confluence
page <https://shadowrobot.atlassian.net/wiki/spaces/SDSR/pages/314310900/The+Cyberglove>`__

There are 3 main users of the CyberGlove. The CyberGlove configuration
is glove and user's hand specific. Therefore, the following branches
exist:

-  A-dg
-  A-penny
-  A-anastasios

-  B-dg
-  B-penny
-  B-anastasios


