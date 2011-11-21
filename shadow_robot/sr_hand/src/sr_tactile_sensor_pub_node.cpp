/**
 * @file   sr_tactile_sensor_pub_node.cpp
 * @author Ugo Cupcic <ugo@ugo-kubuntu.local>
 * @date   Wed Apr  7 15:37:06 2010
 *
 * @brief
 *
 *
 */

#include <ros/ros.h>

#include "sr_hand/sr_tactile_sensor_pub.h"

using namespace shadowhand_tactile_sensor_publisher;

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


/** 
 * The main function initializes the links with the robot, initializes
 * this ROS publisher regularly publishes data
 * regarding the finger tips tactile sensors
 * 
 * @param argc 
 * @param argv 
 * 
 * @return -1 if error linking with the robot (i.e. robot code not started)
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "shadowhand_tactile_sensor_publisher");
  NodeHandle n;

  ShadowhandTactileSensorPublisher shadowhand_pub;

  while( ok() )
    {
      shadowhand_pub.publish();
    }

  return 0;
}
