#include <ros/ros.h>
#include "camera_synchronizer.h"
#include <tf/transform_broadcaster.h>
using namespace ros;

namespace camera_synchronizer{

  CameraSynchronizer::CameraSynchronizer()
 	: n_tilde("~"), timestamp(0.0), publish_rate(0.0)
  {
  
  std::string searched_param;
  std::string path;
  
  //Set publish frequency for sync images
  double frequency;

  n_tilde.param("frequency",frequency,15.0);
  publish_rate = Rate(frequency);  


  //Set publishers and subscribers
  n_tilde.searchParam("left_async",searched_param);
  n_tilde.param(searched_param,path,std::string());
  
  left_sub = node.subscribe(path,10, &CameraSynchronizer::callback_left, this);
  
  n_tilde.searchParam("right_async",searched_param);
  n_tilde.param(searched_param,path,std::string());

  right_sub = node.subscribe(path,10, &CameraSynchronizer::callback_right, this);
  
  n_tilde.searchParam("left_info_async",searched_param);
  n_tilde.param(searched_param,path,std::string());
  
  left_info_sub = node.subscribe(path,10, &CameraSynchronizer::callback_info_left, this);
  
  n_tilde.searchParam("right_info_async",searched_param);
  n_tilde.param(searched_param,path,std::string());

  right_info_sub = node.subscribe(path,10, &CameraSynchronizer::callback_info_right, this);
 
  n_tilde.searchParam("left_sync",searched_param);
  n_tilde.param(searched_param,path,std::string());
  
  left_pub = node.advertise<sensor_msgs::Image>(path,1);

  n_tilde.searchParam("right_sync",searched_param);
  n_tilde.param(searched_param,path,std::string());
  
  right_pub = node.advertise<sensor_msgs::Image>(path,1);

  n_tilde.searchParam("left_info_sync",searched_param);
  n_tilde.param(searched_param,path,std::string());
  
  left_info_pub = node.advertise<sensor_msgs::CameraInfo>(path,1);

  n_tilde.searchParam("right_info_sync",searched_param);
  n_tilde.param(searched_param,path,std::string());
  
  right_info_pub = node.advertise<sensor_msgs::CameraInfo>(path,1);
  }
  
  void CameraSynchronizer::callback_left(const sensor_msgs::ImageConstPtr& msg){
  im_left=convertFromBoost(msg);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(timestamp),"fixed_frame", "test"));
  
  }

  void CameraSynchronizer::callback_right(const sensor_msgs::ImageConstPtr& msg){
 im_right=convertFromBoost(msg);
  }
 
void CameraSynchronizer::callback_info_left(const sensor_msgs::CameraInfoConstPtr& msg){
  info_left=convertFromBoost(msg);
//  info_left.roi.height=240;
//  info_left.roi.width=320;
  }

void CameraSynchronizer::callback_info_right(const sensor_msgs::CameraInfoConstPtr& msg){
  info_right=convertFromBoost(msg);
  int offset;
  n_tilde.param("right_offset", offset,0);
  info_right.roi.x_offset=offset;
//  info_right.roi.height=240;
//  info_right.roi.width=320;
  }


  sensor_msgs::Image CameraSynchronizer::convertFromBoost(const sensor_msgs::ImageConstPtr& msg){
	sensor_msgs::Image ret;
	ret.height=msg->height;
	ret.width=msg->width;
//	ret.header=msg->header;
	ret.header.stamp=ros::Time(timestamp);
//	ret.header.frame_id="/stereo/left/image_raw";
	ret.encoding="rgb8";
	ret.is_bigendian=msg->is_bigendian;
	ret.step=msg->step;
	ret.data=msg->data;
	return ret;
  }

  sensor_msgs::CameraInfo CameraSynchronizer::convertFromBoost(const sensor_msgs::CameraInfoConstPtr& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time(timestamp),"fixed_frame", "test"));
	sensor_msgs::CameraInfo ret;
	ret.height=msg->height;
	ret.width=msg->width;
	ret.roi=msg->roi;
	ret.D=msg->D;
	ret.K=msg->K;
	ret.R=msg->R;
	ret.P=msg->P;
	ret.header.stamp=ros::Time(timestamp);
	ret.header.frame_id="fixed_frame";
	return ret;
  }


  void CameraSynchronizer::publish(){
  left_pub.publish(im_left);
  right_pub.publish(im_right);
  left_info_pub.publish(info_left);
  right_info_pub.publish(info_right);
  timestamp+=1.0;
  ros::spinOnce();
  publish_rate.sleep();
  }

}
