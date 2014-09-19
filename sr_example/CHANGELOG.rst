^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sr_example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* New demo using the new hand_commander
* Remove a potentially dangerous example (colliding fingers). Not a very interesting example either.
* Remove beeps from demo.
* Add tactile sensor data receiver to the HandCommander.
* Allow several concurrent interpolators. Coinciding joints in existing interpolators are overriden by new calls to move_hand.
* Add a python shadowhand Commander (it allows to send commands and read joint positions), and an example script that uses it.
* Tweaks for vagrant demo machine
  * Arg to start without gazebo gui
  * Launch hand sim and rviv view

1.3.0 (2014-02-11)
------------------
* first hydro release

