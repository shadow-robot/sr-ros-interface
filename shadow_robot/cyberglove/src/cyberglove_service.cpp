#include <ros/ros.h>

#include <string>
#include <sstream>

#include "cyberglove/serial_glove.h"
#include "cyberglove/cyberglove_publisher.h"
#include "cyberglove/cyberglove_service.h"

using namespace ros;

namespace cyberglove_service{

CybergloveService::CybergloveService(boost::shared_ptr<cyberglove_publisher::CyberglovePublisher> publish)
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
