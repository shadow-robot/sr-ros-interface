CyberGlove Configuration
========================

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
