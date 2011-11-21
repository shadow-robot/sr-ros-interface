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
 * @brief  Creates an orderly sequence of points when the input is a line.
 *
 *
 */

#ifndef _POINT_SEQUENCE_DETECTION_HPP_
#define _POINT_SEQUENCE_DETECTION_HPP_

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include "pcl/octree/octree.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <kinect_color_segmentation/SurfaceToDremmel.h>
#include <kinect_color_segmentation/WallNormale.h>

namespace sr_kinect
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
  typedef pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> OctreePointCloudChangeDetector;
  typedef pcl::KdTreeFLANN<pcl::PointXYZRGB> KdTreeFLANN;
  typedef pcl::ExtractIndices<pcl::PointXYZRGB> ExtractIndices;

  class PointSequenceDetection
    : public nodelet::Nodelet
  {
  public:
    PointSequenceDetection();
    ~PointSequenceDetection(){};

    virtual void onInit();

    void points_callback(const PointCloud::ConstPtr &cloud);
    void normals_callback(const PointCloudNormal::ConstPtr &cloud);
    bool point_sequence_srv_callback(kinect_color_segmentation::SurfaceToDremmel::Request& request, kinect_color_segmentation::SurfaceToDremmel::Response& response);
    bool surface_normal_srv_callback(kinect_color_segmentation::WallNormale::Request& request, kinect_color_segmentation::WallNormale::Response& response);

  private:
    void read_parameters(ros::NodeHandle & nh);
    unsigned int find_search_point(boost::shared_ptr<PointCloud> cloud);
    unsigned int find_point_index(boost::shared_ptr<PointCloud> cloud, pcl::PointXYZRGB searchPoint);

    ros::Publisher pub_;
    ros::Subscriber points_sub_;
    ros::Subscriber normals_sub_;
    ros::ServiceServer point_sequence_service_;
    ros::ServiceServer surface_normal_service_;

    boost::shared_ptr<PointCloud> output_pcl;
    boost::shared_ptr<PointCloud> srv_output_pcl;
    boost::shared_ptr<PointCloud> previous_pcl;
    
    boost::shared_ptr<PointCloudNormal> srv_output_normals;

    bool first_time;
    std::string line_axis;
    int K;
    
    /** \brief Internal mutex for the point cloud */
    boost::mutex mutex_;
    
    /** \brief Internal mutexfor the normals point cloud */
    boost::mutex mutex_normals_;
  };
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
