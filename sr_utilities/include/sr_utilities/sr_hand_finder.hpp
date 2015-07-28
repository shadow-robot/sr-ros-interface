/*
* File:  sf_hand_finder.hpp
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

#pragma once
#include "ros/ros.h"
namespace shadow_robot
{

class SrHandFinder
{
private:
ros::NodeHandle node_handle_;
public:
  SrHandFinder();
  virtual ~SrHandFinder();
};

} /* namespace stiff_flop */
