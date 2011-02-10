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
#include <std_msgs/Float64.h>

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

namespace shadowrobot
{
/////////////////////////////////
//    CONSTRUCTOR/DESTRUCTOR   //
/////////////////////////////////
SRTactileSensorPublisher::SRTactileSensorPublisher() :
    n_tilde("~"), publish_rate(0.0)
{
    /* We need to attach the program to the robot, or fail if we cannot. */
    if( robot_init() < 0 )
    {
        ROS_FATAL("Robot interface broken\n");
        ROS_BREAK();
    }

    /* We need to attach the program to the hand as well, or fail if we cannot. */
    if( hand_init() < 0 )
    {
        ROS_FATAL("Hand interface broken\n");
        ROS_BREAK();
    }

    // set publish frequency
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 20.0);
    publish_rate = Rate(publish_freq);

    //publishes JointState messages for the robot_state_publisher
    std::string prefix;
    std::string searched_param;
    n_tilde.searchParam("shadowhand_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic_touch = "touch/";
    std::string full_topic_temp = "temp/";
    std::vector<std::string> names;
    names.push_back("ff");
    names.push_back("mf");
    names.push_back("rf");
    names.push_back("lf");
    names.push_back("th");

    sensor_touch_names.push_back("FF_Touch");
    sensor_touch_names.push_back("MF_Touch");
    sensor_touch_names.push_back("RF_Touch");
    sensor_touch_names.push_back("LF_Touch");
    sensor_touch_names.push_back("TH_Touch");

    sensor_temp_names.push_back("FF_Touch_Temp");
    sensor_temp_names.push_back("MF_Touch_Temp");
    sensor_temp_names.push_back("RF_Touch_Temp");
    sensor_temp_names.push_back("LF_Touch_Temp");
    sensor_temp_names.push_back("TH_Touch_Temp");

    for( unsigned int i=0; i<5; ++i)
      {
        std::string top_touch = full_topic_touch + names[i];
	sr_touch_pubs.push_back(n_tilde.advertise<std_msgs::Float64> (top_touch, 20));

        std::string top_temp = full_topic_temp + names[i];
	sr_temp_pubs.push_back(n_tilde.advertise<std_msgs::Float64> (top_temp, 20));
      }
}

/////////////////////////////////
//       PUBLISH METHOD        //
/////////////////////////////////
void SRTactileSensorPublisher::publish()
{
  std_msgs::Float64 msg_temp;
  std_msgs::Float64 msg_touch;

  for( unsigned int i = 0; i < sensor_touch_names.size(); ++i )
    {
        struct sensor s1, s2;
        int res1 = robot_name_to_sensor(sensor_touch_names[i].c_str(), &s1);
        int res2 = robot_name_to_sensor(sensor_temp_names[i].c_str(), &s2);

        /* Check the return value to see if the sensor was found */
        if( res1 )
            ROS_ERROR("Can't open sensor %s\n", sensor_touch_names[i].c_str());
        if( res2 )
            ROS_ERROR("Can't open sensor %s\n", sensor_temp_names[i].c_str());

        msg_touch.data = robot_read_sensor(&s1);
        msg_temp.data = robot_read_sensor(&s2);
        
        sr_touch_pubs[i].publish(msg_touch);
        sr_temp_pubs[i].publish(msg_temp);
    }

    ros::spinOnce();
    publish_rate.sleep();
}

}// end namespace


