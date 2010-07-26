/**
 * @file   shadowhand_to_cybergrasp_remapper_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu May 13 10:39:44 2010
 *
 * @brief
 *
 *
 */

#include <ros/ros.h>

#include "camera_synchronizer.h"

using namespace ros;
using namespace camera_synchronizer;

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_synchronizer");

  CameraSynchronizer sync;
  while(ok()){
    sync.publish();
  }
  return 0;
}
