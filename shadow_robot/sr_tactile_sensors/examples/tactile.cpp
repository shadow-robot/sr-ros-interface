
#include <ros/ros.h>
#include <string>

#include <boost/thread.hpp>

//messages
#include <std_msgs/Float64.h>

//a ros subscriber (will be instantiated later on)
ros::Subscriber sub[5];
std_msgs::Float64::_data_type data[5];
boost::mutex update_mutex;


void callback_ff(const std_msgs::Float64ConstPtr& msg)
{
  update_mutex.lock();
  data[0] = msg->data;
  update_mutex.unlock();
}
void callback_mf(const std_msgs::Float64ConstPtr& msg)
{
  update_mutex.lock();
  data[1] = msg->data;
  update_mutex.unlock();
}
void callback_rf(const std_msgs::Float64ConstPtr& msg)
{
  update_mutex.lock();
  data[2] = msg->data;
  update_mutex.unlock();
}
void callback_lf(const std_msgs::Float64ConstPtr& msg)
{
  update_mutex.lock();
  data[3] = msg->data;
  update_mutex.unlock();
}
void callback_th(const std_msgs::Float64ConstPtr& msg)
{   
  update_mutex.lock();
  data[4] = msg->data;
  update_mutex.unlock();
}


int main(int argc, char** argv)
{
  //init the ros node
  ros::init(argc, argv, "test_tactile");

  ros::NodeHandle node_tactile;

  sub[0] = node_tactile.subscribe("/sr_tactile/touch/ff", 2,  callback_ff);
  sub[1] = node_tactile.subscribe("/sr_tactile/touch/mf", 2,  callback_mf);
  sub[2] = node_tactile.subscribe("/sr_tactile/touch/rf", 2,  callback_rf);
  sub[3] = node_tactile.subscribe("/sr_tactile/touch/lf", 2,  callback_lf);
  sub[4] = node_tactile.subscribe("/sr_tactile/touch/th", 2,  callback_th);
  
  ros::Rate publish_rate = ros::Rate(20.0);
  std_msgs::Float64::_data_type cur_data[5] = {0};

  while( ros::ok() )
  {
    update_mutex.lock();
    for (int i=0; i<5; i++)
      cur_data[i] = data[i];
    update_mutex.unlock();

    ROS_ERROR("TACTILE SENSOR READING: %f %f %f %f %f",
               cur_data[0],
               cur_data[1],
               cur_data[2],
               cur_data[3],
               cur_data[4]);

    publish_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
