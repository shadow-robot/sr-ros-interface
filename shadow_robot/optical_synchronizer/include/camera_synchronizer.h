/**
 * @file   shadowhand_to_cybergrasp_remapper.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu May 13 09:44:52 2010
 *
 * @brief This program remapps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 *
 *
 */

#ifndef   	CAMERA_SYNCHRONIZER_H_
# define   	CAMERA_SYNCHRONIZER_H_
//messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
using namespace ros;

namespace camera_synchronizer{

class CameraSynchronizer
{
 public:
  CameraSynchronizer();
  ~CameraSynchronizer(){};
  void publish();
 private:
  NodeHandle node, n_tilde;
  Rate publish_rate;
  Subscriber left_sub, right_sub, left_info_sub, right_info_sub;
  Publisher left_pub, right_pub, left_info_pub, right_info_pub;
  sensor_msgs::Image im_left, im_right;
  sensor_msgs::CameraInfo info_left, info_right;
  bool first;  
  double timestamp;
  sensor_msgs::Image convertFromBoost(const sensor_msgs::ImageConstPtr& msg);
  sensor_msgs::CameraInfo convertFromBoost(const sensor_msgs::CameraInfoConstPtr& msg);
  /////////////////
  //  CALLBACKS  //
  /////////////////
  //process the jointstates command
  void callback_left(const sensor_msgs::ImageConstPtr& msg);
  void callback_right(const sensor_msgs::ImageConstPtr& msg);
 
  void callback_info_left(const sensor_msgs::CameraInfoConstPtr& msg);
  void callback_info_right(const sensor_msgs::CameraInfoConstPtr& msg);
}; // end class

} //end namespace

#endif 	    /* !CAMERA_SYNCHRONIZER_H_ */
