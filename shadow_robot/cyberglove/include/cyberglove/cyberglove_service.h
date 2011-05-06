/**
 * @file   cyberglove_service.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:25:55 2010
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
 * @brief   A service which can stop / start the Cyberglove publisher.
 *
 *
 */


#ifndef   	CYBERGLOVE_SERVICE_H_
# define   	CYBERGLOVE_SERVICE_H_

#include <ros/ros.h>
#include <vector>
#include "cyberglove_publisher.h"
#include "cyberglove/Start.h"
#include "cyberglove/Calibration.h"
#include <boost/smart_ptr.hpp>

//messages

using namespace ros;

namespace cyberglove{

  class CybergloveService
  {
  public:
    /// Constructor
    CybergloveService(boost::shared_ptr<CyberglovePublisher> publish);
    ~CybergloveService(){};

    //CybergloveService();
    bool start(cyberglove::Start::Request &req, cyberglove::Start::Response &res);
    bool calibration(cyberglove::Calibration::Request &req, cyberglove::Calibration::Response &res);
  private:

    NodeHandle node;
    boost::shared_ptr<CyberglovePublisher> pub;
    ros::ServiceServer service_start;
    ros::ServiceServer service_calibration;
  };

}
#endif
