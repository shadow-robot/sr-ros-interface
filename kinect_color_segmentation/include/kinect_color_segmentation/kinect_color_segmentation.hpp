/**
 * @file   kinect_color_segmentation.hpp
 * @author Toni Oliver <toni@shadowrobot.com>, Ugo Cupcic <ugo@shadowrobot.com>
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
 * @brief  Segment the point cloud filtering points on their RGB values.
 *
 *
 */

#ifndef _KINECT_COLOR_SEGMENTATION_HPP_
#define _KINECT_COLOR_SEGMENTATION_HPP_

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>

namespace sr_kinect
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  /**
   * The KinectColorSegmentation Nodelet segments a point cloud filtering points on their RGB values
   * Publishes an /output topic with the segmented point cloud
   */
  class KinectColorSegmentation
    : public nodelet::Nodelet
  {
  public:
    KinectColorSegmentation();
    ~KinectColorSegmentation(){};

    virtual void onInit();

    /**
     * Callback function for the input cloud topic
     * Filters out the points that doesn't fit in the min max setting for the RGB values
     * @param cloud The input point cloud
     */
    void callback(const PointCloud::ConstPtr &cloud);

  private:
    void read_parameters(ros::NodeHandle & nh);

    ros::Publisher pub_;
    ros::Subscriber sub_;

    boost::shared_ptr<PointCloud> segmented_pcl;

    unsigned int filter_max_r_;
    unsigned int filter_min_r_;
    unsigned int filter_max_g_;
    unsigned int filter_min_g_;
    unsigned int filter_max_b_;
    unsigned int filter_min_b_;
  };
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
