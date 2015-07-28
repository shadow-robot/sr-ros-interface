/*
* File:  sf_hand_finder.cpp
* Author: Vahid Aminzadeh <vahid@shadowrobot.com>
* Copyright 2015 Shadow Robot Company Ltd.
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
* @brief see README.md
*/

#include "sr_utilities/sr_hand_finder.hpp"
#include "ros/ros.h"
#include <string>
namespace shadow_robot
{

SrHandFinder::SrHandFinder()
{
  std::string mapping;
  node_handle_.getParam("hand/mapping", mapping);
  ROS_INFO_STREAM(mapping);
}

SrHandFinder::~SrHandFinder()
{
  // TODO Auto-generated destructor stub
}

} /* namespace stiff_flop */
