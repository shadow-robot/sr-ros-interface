#include <ros/ros.h>

#include <string>
#include <sstream>

#include "cyberglove/serial_glove.h"
#include "cyberglove/cyberglove_publisher.h"
#include "cyberglove/cyberglove_service.h"

using namespace ros;

namespace cyberglove_service{

CybergloveService::CybergloveService(cyberglove_publisher::CyberglovePublisher *publish)
 :  node("~"), pub(publish)
{

  service = node.advertiseService("start",&CybergloveService::start,this);
  service = node.advertiseService("calibration", &CybergloveService::calibration, this);
  ROS_INFO("Listening for service");
}

bool CybergloveService::start(cyberglove::Start::Request &req, cyberglove::Start::Response &res){
    ROS_INFO("Start");
    this->pub->cyberglove_pub.shutdown();
    return true;
}
bool CybergloveService::calibration(cyberglove::Calibration::Request &req, cyberglove::Calibration::Response &res){
    this->pub->isPublishing = false;
    this->pub->initialize_calibration(req.path);
    this->pub->isPublishing = true; 
    return true;
}
}
