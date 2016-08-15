sr\_moveit\_planner\_benchmarking
=================================

A package to test the different MoveIt planners. Planners currently
configured are OMPL, SBPL and STOMP. Different scenes can be found in
the **data/inactive\_tests** folder. To include the scene in the
benchmarking tests, move the yaml file up a directory level into
**data**. The sbpl repository can be found
`here <https://github.com/shadow-robot/sandbox>`__ and stomp
`here <https://github.com/ros-industrial/industrial_moveit>`__. You'll
need to include them in your workspace.

Launch
======

To run all the benchmarkings, simply launch a roscore and run:

::

    rosrun sr_moveit_planner_benchmarking benchmark_planners.py _data:=`rospack find sr_moveit_planner_benchmarking`/data _results:=/tmp

To visualise the tests in Rviz, set the visualisation argument in
launch/benchmarking.launch to 'True'.

Configuration
=============

Currently the configurations for SBPL and STOMP are not generated
automatically as OMPL is. The argument 'generate\_planning\_config' in
`planning\_pipeline.launch.xml <https://github.com/shadow-robot/sr_interface/tree/indigo-devel/sr_multi_moveit/sr_multi_moveit_config/launch/planning_pipeline.launch.xml>`__
is set to True by default but is set to false for the sbpl and stomp
tests.

Different tweaks can be made in the scene yaml files. The first is the
number of planning attempts for each goal and the second being the
planning library name. The STOMP config has been edited to meet the
`suggestions <https://groups.google.com/forum/#!msg/swri-ros-pkg-dev/sNvFmkQsMtg/mGPrXDy8EwAJ>`__,
but probably needs more tweaks.

Notes
=====

ARA\* planner in sbpl currently causes movegroup to crash during the 2nd
planning request. The point cloud for collision\_scene\_2 is generated
from two rosbags of pointcloud data, office\_scene.bag and
office\_scene\_2.bag, `here <data/>`__.
