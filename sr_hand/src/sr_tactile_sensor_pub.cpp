/**
 * @file   sr_tactile_sensor_pub.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 25 15:36:41 2010
 *
 * @brief The goal of this ROS publisher is to publish relevant data
 * concerning the hand at a regular time interval.
 * Those data are (not exhaustive): positions, targets, temperatures,
 * currents, forces, error flags, ...
 *
 *
 */

//ROS include
#include <ros/ros.h>

//messages
#include <shadowhand/joints_data.h>
#include <shadowhand/joint.h>
#include <sensor_msgs/JointState.h>
#include "cybergrasp/cybergraspforces.h"

//generic C/C++ include
#include <vector>
#include <string>
#include <sstream>

//our robot code
#include <robot/robot.h>
#include <robot/hand.h>
#include <robot/hand_protocol.h>

#include "sr_hand/sr_tactile_sensor_pub.h"

using namespace ros;
using namespace std;


namespace shadowhand_tactile_sensor_publisher{
  /////////////////////////////////
  //    CONSTRUCTOR/DESTRUCTOR   //
  /////////////////////////////////
ShadowhandTactileSensorPublisher::ShadowhandTactileSensorPublisher()
  : n_tilde("~"), publish_rate(0.0)
{
  /* We need to attach the program to the robot, or fail if we cannot. */
  if (robot_init()<0)
    {
      ROS_FATAL("Robot interface broken\n");
      ROS_BREAK();
    }
  
  /* We need to attach the program to the hand as well, or fail if we cannot. */
  if (hand_init()<0)
    {
      ROS_FATAL("Hand interface broken\n");
      ROS_BREAK();
    }

  // set publish frequency
  double publish_freq;
  n_tilde.param("publish_frequency", publish_freq, 50.0);
  publish_rate = Rate(publish_freq);

  //publishes JointState messages for the robot_state_publisher
  std::string prefix;
  std::string searched_param;
  n_tilde.searchParam("shadowhand_prefix", searched_param);
  n_tilde.param(searched_param, prefix, std::string());
  std::string full_topic = prefix + "joint_states";
  shadowhand_jointstate_pub = node.advertise<sensor_msgs::JointState>(full_topic, 100);
}


  /////////////////////////////////
  //       PUBLISH METHOD        //
  /////////////////////////////////
void ShadowhandTactileSensorPublisher::publish()
{
  sensor_msgs::JointState jointstate_msg;

  for( unsigned int i = 148 ; i < 174 ; ++i )
    {
      //broken sensor
      if(i == 161 || i == 170)
	continue;
      std::stringstream nametmp;
      nametmp << "finger_sensor_e." << i;
      struct sensor s;
      std::string name = nametmp.str();
      int res=robot_name_to_sensor(name.c_str(), &s);

      /* Check the return value to see if the sensor was found */
      if (res)
	ROS_ERROR("Can't open sensor %s\n", name.c_str());
      
      jointstate_msg.name.push_back(name);
      jointstate_msg.effort.push_back(robot_read_sensor( &s ));
    }

  //publish JointState message
  shadowhand_jointstate_pub.publish(jointstate_msg);

  ros::spinOnce();
  publish_rate.sleep();
}

}// end namespace


