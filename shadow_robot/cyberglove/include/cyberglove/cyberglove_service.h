/**
 * @file   cyberglove_service.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:25:55 2010
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

namespace cyberglove_service{

  class CybergloveService
  {
  public:
    /// Constructor
    CybergloveService(boost::shared_ptr<cyberglove_publisher::CyberglovePublisher> publish);
    ~CybergloveService(){};

    //CybergloveService();
    bool start(cyberglove::Start::Request &req, cyberglove::Start::Response &res);
    bool calibration(cyberglove::Calibration::Request &req, cyberglove::Calibration::Response &res);
  private:
    
    NodeHandle node;
    boost::shared_ptr<cyberglove_publisher::CyberglovePublisher> pub;
    ros::ServiceServer service_start;
    ros::ServiceServer service_calibration;
  };

}
#endif
