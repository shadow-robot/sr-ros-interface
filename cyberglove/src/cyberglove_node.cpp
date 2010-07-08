/**
 * @file   cyberglove_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Apr 22 10:21:50 2010
 * 
 * @brief  The cyberglove node publishes data collected from a
 * Cyberglove.
 * 
 * 
 */

#include <ros/ros.h>

#include "cyberglove/cyberglove_publisher.h"
#include "cyberglove/cyberglove_service.h"
#include "cyberglove/Start.h"
using namespace cyberglove_publisher;
using namespace cyberglove_service;



/////////////////////////////////
//           MAIN              //
/////////////////////////////////


/** 
 *  Start the cyberglove publisher.
 *
 * @param argc 
 * @param argv 
 * 
 * @return -1 if error (e.g. no glove found)
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cyberglove_publisher");
  //NodeHandle n;

  CyberglovePublisher *cyberglove_pub = new CyberglovePublisher();
  CybergloveService service(cyberglove_pub);  

  while( ok() )
    {
      cyberglove_pub->publish();
    }

  return 0;
}
