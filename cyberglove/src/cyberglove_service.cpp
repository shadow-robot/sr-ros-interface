/**
 * @file   cyberglove_service.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Mar 29 15:04:02 2011
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
 * @brief  Adding services to the cyberglove, to be able to reload the
 * calibration mostly.
 *
 *
 */

#include <ros/ros.h>

#include <string>
#include <sstream>

#include "cyberglove/cyberglove_publisher.h"
#include "cyberglove/cyberglove_service.h"

using namespace ros;

namespace cyberglove{

CybergloveService::CybergloveService(boost::shared_ptr<CyberglovePublisher> publish)
 :  node("~"), pub(publish)
{
  service_start = node.advertiseService("start",&CybergloveService::start,this);
  service_calibration = node.advertiseService("calibration", &CybergloveService::calibration, this);
  ROS_INFO("Listening for service");
}

bool CybergloveService::start(cyberglove::Start::Request &req, cyberglove::Start::Response &res){
    if(req.start){
        ROS_INFO("Glove is now publishing");
        this->pub->setPublishing(true);
    }
    else{
        ROS_INFO("Glove has stopped publishing");
        this->pub->setPublishing(false);
    }
    //this->pub->cyberglove_pub.shutdown();
    return true;
}
bool CybergloveService::calibration(cyberglove::Calibration::Request &req, cyberglove::Calibration::Response &res){
    this->pub->setPublishing(false);
    this->pub->initialize_calibration(req.path);
    this->pub->setPublishing(true);
    return true;
}
}
