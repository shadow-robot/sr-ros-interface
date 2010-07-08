/**
 * @file   motor_info_publisher.h
 * @author Nicolas Goossaert <nicolas@shadowrobot.com>
 * @date   Thu May 20 11:26:41 2010
 * 
 * @brief  header of the motor_info_publisher.cpp file
 * 
 * 
 */


#ifndef MOTOR_INFO_PUBLISHER_H_
#define MOTOR_INFO_PUBLISHER_H_

#include <ros/ros.h>
#include "sr_display/motor_data.h"

//messages
#include <sr_hand/joint.h>
#include <sr_hand/joints_data.h>


using namespace ros;
using namespace std;

namespace motor_info_publisher{

class MotorInfoPublisher
{
public:
	/// Constructor
	MotorInfoPublisher();

	/// Destructor
	~MotorInfoPublisher();

	/// Publishes all the markers
	void publish();

	/// The number of joints in the hand
	static const int number_hand_joints;

	/// A vector containing all the names of the joints
	vector<string> joints_names;

	/// A Vector containing MotorData, each of them representing a motor and containing all the data about this motor
	vector<motor_data::MotorData> datatest;

	/// Initializes the name of each joint
	void init_names();

private:

	/// ROS Node Handle
	NodeHandle node, n_tilde;

	/// The frequency of publication of the markers
	Rate publish_rate;

	/// The publisher of the markers
	Publisher motor_info_pub;

	/// The subscriber used to update the data about the hand
	Subscriber shadowhand_data_sub;

	/**The Callback Method of the Subscriber, defining what to do each time data are received
	 * @param msg the message received by the subscriber
	 */
	void jointsDataCallback(const sr_hand::joints_dataConstPtr& msg);


}; // end class MotorInfoPublisher

} // end namespace

#endif
