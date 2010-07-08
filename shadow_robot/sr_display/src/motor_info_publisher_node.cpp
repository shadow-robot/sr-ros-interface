/**
 * @file   motor_info_publisher_node.cpp
 * @author Nicolas Goossaert <nicolas@shadowrobot.com>
 * @date   Thu May  20 12:16:06 2010
 *
 * @brief This node sends the text markers displaying the
 * informations about the robot to the specified topic. It
 * also launches the service allowing to display or not
 * each information about each motor individually.
 *
 *
 */


#include <ros/ros.h>

#include <sr_display/display_check.h>

#include "sr_display/motor_info_publisher.h"

using namespace motor_info_publisher;

MotorInfoPublisher* motor_info_pub;
display_check::DisplayCheck* dc;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_info_publisher");
  NodeHandle n;

  motor_info_pub = new MotorInfoPublisher();
  dc = new display_check::DisplayCheck(motor_info_pub);

  while( ok() )
    {
	  ros::spinOnce();
	  motor_info_pub->publish();
    }

  if(motor_info_pub != NULL)
	  delete motor_info_pub;
  if(dc != NULL)
	  delete dc;

  return 0;
}
