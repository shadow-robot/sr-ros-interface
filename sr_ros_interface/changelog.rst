^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sr_ros_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2013-12-19)
------------------
* Port to hydro with catkin

1.1.0 (2012-08-20)
------------------
* New organisation: the urdf and descriptions are regrouped in the new sr\_description stack. Numerous bug fixes.

1.0.1 (2012-08-20)
------------------
* Resetting the controllers when restarting them (zeroing motor strain gauges)

1.0.0 (2012-08-09)
------------------
* Better mixed position velocity controllers, Hand with Biotacs model added, Reorganised the code in different stacks. Removed some packages from the shadow\_robot stack to put them in either sr\_contrib, sr\_config, sr\_teleop, sr\_visualisation, sr\_demo stacks. We're keeping only the core packages in this stack for easier release / maintenance.
