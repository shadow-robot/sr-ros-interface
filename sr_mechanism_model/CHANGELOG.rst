^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sr_mechanism_model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2015-04-07)
------------------
* fix for joint names of bimanual systems

1.3.1 (2014-07-18)
------------------

1.3.0 (2014-02-11)
------------------
* fixed sr_kinematics
* modified how library dependencies are forwarded
* used world files from sr_grasp_stabilization. resolved deprecation messages
* added some message dependencies as recommended by wiki
* fixed build issues that appeared in other PCs
* Now the test build but don't all succeed yet
* corrections
* added install directives and removed old-fashioned bin folder
* removed ROS_BUILT_TYPE which is unrecommended. Also removed a few redundant lines
* almost buildable version
* another step towards catkining
* first step towards catkinizing
* Added the muscle hand transmissions
* Created new simple and joint0 transmission classes for the muscle actuators
* modified mainpages / manifests / ... for cleaner doc
* Added a mean to compile with tinyxml without changing the cpp or hpp files
* Trivial
* trivial
* Fixed joint 0 transmission for Gazebo
* removed the mechanical reduction from the transmissions (it doesn't change much as this factor was set to 1, but it simplifies the code).
  added alpha beta filter to the sr_math_utils
  (lp:863118)
* reorganizing our 2 repositories: we want the shadow_robot stack to be working on its own. the shadow_robot_etherCAT stack should contain only etherCAT specific code.
* reorganizing our 2 repositories: we want the shadow_robot stack to be working on its own. the shadow_robot_etherCAT stack should contain only etherCAT specific code.
* reorganizing our 2 repositories: we want the shadow_robot stack to be working on its own. the shadow_robot_etherCAT stack should contain only etherCAT specific code.
* trying to get the sr_edc_mechanism_controllers to work with gazebo.
* trivial
* fixed joint 0s controllers.
* Updating the documentation.
* Splitting the tree in two different trees: we want one bzr repository per branch.
* Contributors: GuiHome, Toni Oliver, Ugo Cupcic, shadowmanos
