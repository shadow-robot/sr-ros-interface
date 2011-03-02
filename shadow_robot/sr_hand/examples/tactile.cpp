
#include <ros/ros.h>
#include <string>

//messages
#include <std_msgs/Float64.h>

//a ros subscriber (will be instantiated later on)
ros::Subscriber sub;


void callback(const std_msgs::Float64ConstPtr& msg)
{   
  ROS_ERROR("TACTILE SENSOR READING: %f", msg->data);
}


int main(int argc, char** argv)
{
  //init the ros node
  ros::init(argc, argv, "test_tactile");

  ros::NodeHandle node_tactile;

  sub = node_tactile.subscribe("/sr_tactile/touch/ff", 2,  callback);
  
  ros::spin();

  return 0;
}
