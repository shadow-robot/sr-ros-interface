/**
 * @file   optical_dataglove.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Jul  7 18:07:42 2010
 * 
 * @brief  The Optical Dataglove class
 * 
 * 
 */

#include "optical_dataglove/optical_dataglove.h"

namespace optical_dataglove
{
  OpticalDataglove::OpticalDataglove (ros::NodeHandle & nh):nh_ (nh), it_ (nh_)
  {
    // Advertise image messages to a topic
    image_pub_ = it_.advertise ("/demo/output_image", 1);
    // Listen for image messages on a topic and setup callback
    image_sub_ = it_.subscribe ("/usb_cam/image_raw", 1, &OpticalDataglove::imageCallback, this);
    // Open HighGUI Window
    cv::namedWindow ("hsv", 1);
  }

  void OpticalDataglove::imageCallback (const sensor_msgs::ImageConstPtr & msg_ptr)
  {
    // Convert ROS Imput Image Message to IplImage
    try
      {
	cv_input_ = bridge_.imgMsgToCv (msg_ptr, "bgr8");
      }
    catch (sensor_msgs::CvBridgeException error)
      {
	ROS_ERROR ("CvBridge Input Error");
      }

    // Convert IplImage to cv::Mat
    img_in_ = cv::Mat (cv_input_).clone ();
    // Convert Input image from BGR to HSV
    cv::cvtColor (img_in_, img_hsv_, CV_BGR2HSV);

    // Display HSV Image in HighGUI window
    cv::imshow ("hsv", img_hsv_);

    // Needed to  keep the HighGUI window open
    cv::waitKey (3);

    // Convert cv::Mat to IplImage
    cv_output_ = img_hsv_;
    // Convert IplImage to ROS Output Image Message and Publish
    try
      {
	image_pub_.publish (bridge_.cvToImgMsg (&cv_output_, "bgr8"));
      }
    catch (sensor_msgs::CvBridgeException error)
      {
	ROS_ERROR ("CvBridge Output error");
      }
  }
}; //end namespace
