/**
 * @file   display_check.h
 * @author Nicolas Goossaert <nicolas@shadowrobot.com>
 * @date   Thu Jun 10 16:33:20 2010
 * 
 * @brief  header of the display_check.cpp file
 * 
 * 
 */

#ifndef DISPLAY_CHECK_H_
#define DISPLAY_CHECK_H_

#include <ros/ros.h>
#include <vector>
#include <string>

#include <sr_display/motor_info_publisher.h>

#include "../srv_gen/cpp/include/sr_display/display_check.h"

using namespace ros;
using namespace std;

namespace display_check {

class DisplayCheck {

public:

	/**Constructor
	 * @param pub the MotorInfoPublisher that will publish the informations
	 */
	DisplayCheck(motor_info_publisher::MotorInfoPublisher* pub);

	/// Destructor
	~DisplayCheck();

	/**Allows the display of the concerned attribute
	 * @param req ROS service Request
	 * @param res ROS service Response
	 */
	bool setDisplay(sr_display::display_check::Request &req, sr_display::display_check::Response &res);

private:

	/// The MotorInfoPublisher that will publish the informations
	motor_info_publisher::MotorInfoPublisher* mip;

	/// ROS services (server part)
	ros::ServiceServer service_true, service_false;

	/// ROS Node Handle
	NodeHandle node;

}; // end class DisplayCheck

} // end namespace

#endif
