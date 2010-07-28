/*
 * display_check_client.cpp
 *
 *  Created on: 22 juin 2010
 *      Author: hand
 */

#include "ros/ros.h"
#include "../srv_gen/cpp/include/sr_display/display_check.h"
#include <sr_display/display_check.h>
#include <sr_display/motor_info_publisher.h>
#include <string>
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "display_check_client");

	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<display_check::DisplayCheck>("display_check");

	//display_check::DisplayCheck* srv = new display_check::DisplayCheck(motor_info_pub);
	sr_display::display_check srv;

	srv.request.joint_name = argv[1];
	srv.request.attr_name = argv[2];
	srv.request.display = atoll(argv[3]);

	if (client.call(srv))
	{
		stringstream s;
		s << srv.request.joint_name;
		s << ":";
		s << srv.request.attr_name;
		s << " display set to ";
		s << srv.request.display;
		string texte = s.str();
		ROS_INFO("blabla");
	}
	else
	{
		ROS_ERROR("Failed to call service display_check");
		return 1;
	}

	return 0;
}
