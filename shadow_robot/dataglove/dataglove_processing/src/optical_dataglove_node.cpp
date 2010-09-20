/**
 * @file   optical_dataglove_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Jul  7 18:02:38 2010
 * 
 * @brief  Contains the main for the optical dataglove. Start a ROS node.
 * 
 * 
 */


#include <boost/smart_ptr.hpp>
#include "optical_dataglove/message_publisher.h"
#include "optical_dataglove/data_analyser.h"
#include "optical_dataglove/two_dimensions.h"
#include "optical_dataglove/position_mapper.h"
#include "optical_dataglove/reverse_kinematics.h"
#include "optical_dataglove/optical_dataglove.h"

using namespace opticaldataglove;

int main (int argc, char **argv)
{
  // Initialize ROS Node
  ros::init (argc, argv, "optical_dataglove");
  // Start node and create a Node Handle
  ros::NodeHandle nh;
  // Instantiate 
  //boost::shared_ptr<OpticalDataglove> optical_dataglove (new OpticalDataglove(nh));
  boost::shared_ptr<DataAnalyser> data (new TwoDimensionsAnalyser());
  boost::shared_ptr<PositionMapper> pos (new ReverseKinematics(data));
  boost::shared_ptr<MessagePublisher> mes (new MessagePublisher(pos));
  //mes->start();
  // Spin ...
  ros::Rate loop_rate(5);
  
  while(ros::ok()){
    //ROS_INFO("ros::ok()");
    mes->publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin ();
  // ... until done
  return 0;
}
