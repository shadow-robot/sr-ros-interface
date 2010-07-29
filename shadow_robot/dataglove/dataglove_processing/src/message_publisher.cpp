#include "optical_dataglove/message_publisher.h"

namespace opticaldataglove
{

MessagePublisher::MessagePublisher(boost::shared_ptr<PositionMapper> position_mapper): publishedTopic("joint_state"), node("~"), positionMapper(position_mapper), isPublishing(true)
{
     service_start = node.advertiseService("start", &MessagePublisher::start, this);
     service_stop = node.advertiseService("stop", &MessagePublisher::stop, this);
    publisher = node.advertise<sensor_msgs::JointState>("joint_state",2); 
    ROS_INFO("Ready to start");
}

MessagePublisher::~MessagePublisher()
{

}

bool MessagePublisher::start(opticaldataglove::Start::Request &req, opticaldataglove::Start::Response &res)
{
    ROS_INFO("Starting publisher");
    isPublishing = true;
    res.state = true;
    return true;
}

bool MessagePublisher::stop(opticaldataglove::Stop::Request &req, opticaldataglove::Stop::Response &res)
{
    ROS_INFO("Stoping publisher");
    isPublishing = false;
    res.state = false;
    return true;
}

void MessagePublisher::publish()
{
   if (isPublishing){ 
        std::map<std::string, JointData> map = this->positionMapper->getHandPositions();
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();

        for(std::map<std::string, JointData>::const_iterator it = map.begin(); it != map.end(); it++)
        {
            JointData value = it->second;
            joint_state.name.push_back(it->first);
            joint_state.position.push_back(value.position);
            joint_state.velocity.push_back(0.0);
            joint_state.effort.push_back(0.0);
        }

        this->publisher.publish(joint_state);
    }
}

}
