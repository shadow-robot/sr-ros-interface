^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sr_hand
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2015-04-07)
------------------
* starting trajectory controllers only for the arm when running arm + hand as it is more standard. Can't have both running with ros_control
* removing cartesian control - not implemented in ros-control + changing to the new traj controller
* Added shutdown-timeout=1.0 in controller spawner to improve shutdown time
  Added a boolean to stop spinner in gazebo controller manager plugin to improve shutdown
* Better ethercat compatibility
* Add arguments to allow launching a particular robot model on gazebo.
* Moved gnuplot as a run dependency since sr_self_test is also running outside the gtest
  Fixed missing image_path
* mod debug mode of launch file
  added args for launching gazebo
  renamed joint variable in python because it shadowed import
* deleting deprecated rviz config
* adding the standard ros topics to our old hand interface - useulf for the virtual hand

1.3.1 (2014-07-18)
------------------

1.3.0 (2014-02-11)
------------------
* first hydro release

