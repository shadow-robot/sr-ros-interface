/**
 * @file   3D_mouse_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jul 14 10:27:51 2010
 * 
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
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
