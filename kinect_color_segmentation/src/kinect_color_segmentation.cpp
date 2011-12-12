/**
 * @file   kinect_color_segmentation.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Toni Oliver <toni@shadowrobot.com>
 * @date   Mon Nov  7 14:58:17 2011
 *
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
 * @brief  Segment the point cloud.
 *
 *
 */

#include <kinect_color_segmentation/kinect_color_segmentation.hpp>

namespace sr_kinect
{
  PLUGINLIB_DECLARE_CLASS(sr_kinect, KinectColorSegmentation, sr_kinect::KinectColorSegmentation, nodelet::Nodelet);

  KinectColorSegmentation::KinectColorSegmentation()
    : nodelet::Nodelet(), filter_max_r_(255), filter_min_r_(0), filter_max_g_(255), filter_min_g_(0), filter_max_b_(255), filter_min_b_(0)
  {}

  void KinectColorSegmentation::onInit()
  {
    ros::NodeHandle & nh = getNodeHandle();
    sub_ = nh.subscribe<PointCloud >("cloud_in",2, &KinectColorSegmentation::callback, this);
    pub_ = nh.advertise<PointCloud>(this->getName() + "/output", 1000);
    segmented_pcl = boost::shared_ptr<PointCloud>(new PointCloud() );

    read_parameters(nh);
  }

  void KinectColorSegmentation::callback(const PointCloud::ConstPtr &cloud)
  {
    segmented_pcl->clear();

    for(unsigned int i=0; i < cloud->size(); ++i)
    {
        if((( cloud->at(i).r) > filter_min_r_)
           && (( cloud->at(i).r) < filter_max_r_)
           &&(( cloud->at(i).g) > filter_min_g_)
           &&(( cloud->at(i).g) < filter_max_g_)
           &&(( cloud->at(i).b) > filter_min_b_)
           &&(( cloud->at(i).b) < filter_max_b_)
          )
          segmented_pcl->push_back( cloud->at(i) );
    }
    segmented_pcl->header = cloud->header;

    pub_.publish(segmented_pcl);
  }

  void KinectColorSegmentation::read_parameters(ros::NodeHandle & nh)
  {
    //Parameter reading
    std::string base_name = nh.resolveName(this->getName(), true);
    int param_read = 0;
    if (nh.getParam(base_name + "/filter_max_r", param_read))
    {
      filter_max_r_ = static_cast<unsigned int>(param_read);
      NODELET_INFO_STREAM("Read max r: " << filter_max_r_);
    }
    if (nh.getParam(base_name + "/filter_min_r", param_read))
    {
      filter_min_r_ = static_cast<unsigned int>(param_read);
      NODELET_INFO_STREAM("Read min r: " << filter_min_r_);
    }
    if (nh.getParam(base_name + "/filter_max_g", param_read))
    {
      filter_max_g_ = static_cast<unsigned int>(param_read);
      NODELET_INFO_STREAM("Read max g: " << filter_max_g_);
    }
    if (nh.getParam(base_name + "/filter_min_g", param_read))
    {
      filter_min_g_ = static_cast<unsigned int>(param_read);
      NODELET_INFO_STREAM("Read min g: " << filter_min_g_);
    }
    if (nh.getParam(base_name + "/filter_max_b", param_read))
    {
      filter_max_b_ = static_cast<unsigned int>(param_read);
      NODELET_INFO_STREAM("Read max b: " << filter_max_b_);
    }
    if (nh.getParam(base_name + "/filter_min_b", param_read))
    {
      filter_min_b_ = static_cast<unsigned int>(param_read);
      NODELET_INFO_STREAM("Read min b: " << filter_min_b_);
    }
  }
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


