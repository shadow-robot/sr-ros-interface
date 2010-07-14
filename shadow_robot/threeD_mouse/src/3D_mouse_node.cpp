/**
 * @file   3D_mouse_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jul 14 10:27:51 2010
 * 
 * @brief  
 * 
 * 
 */

#include <ros/ros.h>

#include "threeD_mouse/3D_mouse.h"

using namespace threedmouse;


int main( int argc, char** argv )
{
  ros::init(argc, argv, "threedmouse");
  ros::NodeHandle n;

  ThreeDMouse threedmouse;

  while( ros::ok() )
    {
      threedmouse.publish();
    }

  return 0;
}
