^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sr_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2015-04-07)
------------------
* Inertia now exact values, with inertia tensor at COM (gazebo and ode support the small values now)
  Removed explicit viscous damping from sr_gazebo_plugin,
  enabled springDamper in each joint (implicit viscous damping)
  changed simulation parameters for stability
  and retuned the controllers, explicitely define friction_deadband for simulated controllers (default is for real hand and is too high)
* Added shutdown-timeout=1.0 in controller spawner to improve shutdown time
  Added a boolean to stop spinner in gazebo controller manager plugin to improve shutdown
* using prefix on joints didn't work with gazebo controllers. This is now working

1.3.1 (2014-07-18)
------------------

1.3.0 (2014-02-11)
------------------
* first hydro release

