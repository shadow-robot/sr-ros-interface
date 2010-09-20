#ifndef MESSAGE_PUBLISHER_H_
#define MESSAGE_PUBLISHER_H_

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include "opticaldataglove/Start.h"
#include "opticaldataglove/Stop.h"
#include "optical_dataglove/joint_data.h"
#include "optical_dataglove/position_mapper.h"
#include <boost/smart_ptr.hpp>
#include <sensor_msgs/JointState.h>

namespace opticaldataglove
{

/*
 * Interfacing the acquisition and computing process with the hand over ROS
 */
class MessagePublisher 
{
public:

    MessagePublisher(boost::shared_ptr<PositionMapper> position_mapper);


    ~MessagePublisher();

    std::string publishedTopic;
    
/*
 * Publishes the position of the hand on the specified topic
 */
    void publish();
/*
 * ROS service to start the publishing
 */
    bool start(opticaldataglove::Start::Request &req, opticaldataglove::Start::Response &res);
/*
 * ROS service to stop the publishing
 */
    bool stop(opticaldataglove::Stop::Request &req, opticaldataglove::Stop::Response &res);

private:
    
   std::map<std::string, JointData> handPositions;
   ros::NodeHandle node;
   boost::shared_ptr<PositionMapper> positionMapper;
   ros::Publisher publisher;
   ros::ServiceServer service_start;
   ros::ServiceServer service_stop;
   bool isPublishing;    
};
}
#endif //MESSAGE_PUBLISHER_H_
