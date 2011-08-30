
First of all, don't forget to edit your ROS_PACKAGE_PATH variable. To do this,
edit your ~/.bashrc and AFTER the source /path/to/ros/setup.sh add: 

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/path/to/shadow_robot

where /path/to/shadow_robot is the complete path to the shadow_robot directory you 
got from the bazaar repository.

To start using this ROS interface, the best way to go is to generate the doc:

$ cd doc
$ ./generate_doc.sh

Then fire up your favourite browser and open the generated rosdoc/index.html.
You can start by reading the sr_hand documentation.

