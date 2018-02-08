optoforce
=========

ROS driver for the `Optoforce
sensor <http://optoforce.com/3dsensor/>`__.

Optoforce models supported
--------------------------

-  Single-channel 3 axis force sensor
-  Multi-channel 3 axis force sensor (4 channels)
-  Single-channel 6 axis force sensor

Driver description
------------------

The driver relies on the pySerial library to get data from the sensors
through USB. The module optoforce is completely independant from ROS and
can be used as such.

The ROS node built with this driver publishes messages of type
``geometry_msgs/WrenchStamped`` for every sensor.

Installation of udev rule
-------------------------

Why ? Thanks to this udev rule, the sensors' names will no longer depend
on the order in which they are plugged in the computer. The names would
look like ``/dev/optoforce_ISE174``.

To do so, copy ``optoforce.rules`` to ``/etc/udev/rules.d/``. Then, open
this file and replace ``PATH/TO`` with the actual path to the
``get_serial.py`` file.

Also make sure that the file ``src/optoforce/get_serial.py`` has
execution rights for all users.

Now, next time an optoforce sensor is plugged in, it should appear in
``/dev/`` as ``optoforce_SERIAL`` where ``SERIAL`` is the actual
sensor's serial number.

    **Note:** The path to give to the optoforce node has to be adapted.

    **Note:** Should there be a problem getting the serial number, a
    random number will be used instead of the serial number.

Quickstart
----------

Have a look at ``optoforce.launch`` for the node's parameters and their
values.

It also loads parameters from the
``multi_channel_3_axis_generic_scale.yaml`` file. They are ratios from
raw data to Newton, as found in the sensor's sensitivity report. If you
have a single channel force sensor, have a look at
``single_channel_3_axis_generic_scale.yaml``.

Shadow robot's hand
~~~~~~~~~~~~~~~~~~~

-  ``optoforce_hand.launch`` start the optoforce node configured for the
   hand
-  ``rviz.launch`` will start Rviz configured to display the hand

