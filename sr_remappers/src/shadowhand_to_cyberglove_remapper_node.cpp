/**
 * @file   shadowhand_to_cyberglove_remapper_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu May 13 10:39:44 2010
 *
 * @brief Launch a ros node to remap data coming from the Cyberglove to the Dextrous Hand.
 *
 *
 */

#include <ros/ros.h>

#include "sr_remappers/shadowhand_to_cyberglove_remapper.h"

using namespace ros;
using namespace shadowhand_to_cyberglove_remapper;

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cyberglove_remapper");

  ShadowhandToCybergloveRemapper remapper;
  ros::spin();

  return 0;
}
