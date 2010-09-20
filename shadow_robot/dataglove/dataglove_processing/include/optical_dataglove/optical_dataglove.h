/**
 * @file   optical_dataglove.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Jul  7 18:08:41 2010
 * 
 * @brief  The Optical Dataglove Class.
 * 
 * 
 */

#ifndef OPTICAL_DATAGLOVE_H_
#define OPTICAL_DATAGLOVE_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>

namespace optical_dataglove
{
  class OpticalDataglove 
  {
  public:
    OpticalDataglove(ros::NodeHandle & nh);
    ~OpticalDataglove(){};

  protected:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    sensor_msgs::CvBridge bridge_;
    image_transport::Publisher image_pub_;
    cv::Mat img_in_;
    cv::Mat img_hsv_;
    IplImage *cv_input_;
    IplImage cv_output_;

    void imageCallback (const sensor_msgs::ImageConstPtr & msg_ptr);
  };
};

#endif      /* !  OPTICAL_DATAGLOVE_H_  */
