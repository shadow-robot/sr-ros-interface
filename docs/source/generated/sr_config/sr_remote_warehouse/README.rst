Shadow Pose Warehouse
=====================

This package allows connection to a mongodb warehouse for storing robot
poses for use with the Shadow Robot Hand.

It provides a cloud alternative to local a mongo warehouse as provided
by `mongodb <http://wiki.ros.org/mongodb>`__. As for the local
warehouse, the remote one also provides storage for constraints and
planning scenes. These features aren't used by our code, and so will be
left unexplained here.

Launching
---------

The easiest way to launch the database connection is with
``roslaunch sr_remote_warehouse remote_warehous.launch`` This will add
relevant parameters to the param server, then launch a
``warehouse_ros warehouse_services`` node, providing access to robot
states saved in the database. See below for further explanation of this.

Add ``launch_services:=false`` to the launch command if you don't want
the services for some reason.

Connection details
------------------

Default access is read only, using the details stored in
``sr_remote_warehouse/launch/remote_warehouse.yaml``

If you want write access for some reason (and we'll always be grateful
for contributions :), please `contact
us <mailto:software@shadowrobot.com>`__.

Accessing Poses
---------------

Services
~~~~~~~~

The poses stored in the database can be most easily accessed using the
following services, launched by default with the database connection.

-  ``/list_robot_states`` returns a list of the names of the robot
   states available in the db. If you provide a ``robot`` argument, only
   poses for the named robot are returned.
-  ``/has_robot_state`` returns a bool describing if the named state is
   available. If ``robot`` is provided, the service returns true only if
   the named pose is available, and for the named robot.
-  ``/get_robot_state`` returns a ``moveit_msgs/RobotState`` containing
   the saved pose. The ``robot`` argument can be used to filter poses
   only for the named robot as before.

Rviz
~~~~

If the database connection has been launched, the rviz motion planning
plugin can connect in the same way as it would to a local db, although
obviously read only access.

Other
~~~~~

Support for transparent access from ``sr_robot_commander``,
``sr_grasp_controller`` and eventually directly from
``move_group_commander`` Watch this space for further details.

Cloning Locally
---------------

The facility to clone the warehouse to be used locally is scheduled to
be added soon.
