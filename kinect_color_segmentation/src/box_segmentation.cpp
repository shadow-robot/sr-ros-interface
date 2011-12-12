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

#include <kinect_color_segmentation/box_segmentation.hpp>
#include <pcl/common/common.h>

#include <object_manipulation_msgs/ClusterBoundingBox.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <pcl_ros/transforms.h>
//#include <pcl/exceptions.h>

namespace sr_kinect
{
PLUGINLIB_DECLARE_CLASS(sr_kinect, BoxSegmentation, sr_kinect::BoxSegmentation, nodelet::Nodelet);

BoxSegmentation::BoxSegmentation() :
    nodelet::Nodelet()
{
}

void BoxSegmentation::onInit()
{
  ros::NodeHandle & nh = getNodeHandle();
  sub_ = nh.subscribe < PointCloud > ("cloud_in", 2, &BoxSegmentation::callback, this);
  sub2_ = nh.subscribe < PointCloud > ("plane", 2, &BoxSegmentation::callback2, this);
  sub3_ = nh.subscribe < pcl::ModelCoefficients > ("plane_coeff", 2, &BoxSegmentation::callback3, this);
  pub_ = nh.advertise < PointCloud > (this->getName() + "/output", 1000);
  segmented_pcl = boost::shared_ptr < PointCloud > (new PointCloud());
  plane_coefficients = boost::shared_ptr < pcl::ModelCoefficients > (new pcl::ModelCoefficients());

//  find_cluster_bounding_box_client = nh.serviceClient < object_manipulation_msgs::FindClusterBoundingBox
//      > ("/find_cluster_bounding_box2");
  
  //Initialize plane coefficients to coincide with z=0 plane
//  plane_coefficients->values.clear();
//  plane_coefficients->values.push_back(0.0);
//  plane_coefficients->values.push_back(0.0);
//  plane_coefficients->values.push_back(1.0);
//  plane_coefficients->values.push_back(0.0);
  
  min_pt_[0] = 0.0;
  min_pt_[1] = 0.0;
  min_pt_[2] = 0.0;
  min_pt_[3] = 0.0;
  max_pt_[0] = 1.0;
  max_pt_[1] = 1.0;
  max_pt_[2] = 1.0;
  max_pt_[3] = 0.0;
  

  //initialize the tf listener and broadcaster
//  tf_listener = boost::shared_ptr < tf::TransformListener > (new tf::TransformListener());
//  tf_broadcaster = boost::shared_ptr < tf::TransformBroadcaster > (new tf::TransformBroadcaster());

  read_parameters(nh);
}

void BoxSegmentation::callback(const PointCloud::ConstPtr &cloud)
{
  segmented_pcl->clear();
  
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  
  try
  {
    pcl::getPointsInBox(*cloud, min_pt_, max_pt_, inliers->indices);    
  }
  catch(pcl::IsNotDenseException exc)
  {
    ROS_ERROR("getPointsInBox Is Not Dense exception");
  }
  
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*segmented_pcl);
      
  segmented_pcl->header = cloud->header;
  segmented_pcl->is_dense = cloud->is_dense;

  if(segmented_pcl->size() > 0)
  {
    pub_.publish(segmented_pcl);
  }
}

void BoxSegmentation::callback2(const PointCloud::ConstPtr &cloud)
{
//  sensor_msgs::PointCloud2 cloud2;
//  pcl::toROSMsg(*cloud, cloud2);
//  sensor_msgs::PointCloud cloud3;
//  sensor_msgs::convertPointCloud2ToPointCloud(cloud2, cloud3);
//
//  object_manipulation_msgs::FindClusterBoundingBox srv;
//  object_manipulation_msgs::ClusterBoundingBox bounding_box;
//  srv.request.cluster = cloud3;
//  ROS_DEBUG_STREAM("Cluster for the unknown object" << srv.request.cluster);
//
//  if (find_cluster_bounding_box_client.call(srv) == 1)
//  {
//    ROS_DEBUG_STREAM("Response Pose " << srv.response.pose << "Response Box Dims " << srv.response.box_dims);
//
//    bounding_box.pose_stamped = srv.response.pose;
//    bounding_box.dimensions = srv.response.box_dims;
//  }
//  else
//  {
//    ROS_ERROR("Failed to get the bounding box for the unknown object");
//  }
//
//  //Publish transformation to the bounding box frame
//  tf::Quaternion tmp_quat;
////      tmp_quat.x = bounding_box.pose_stamped.pose.orientation.x;
////      tmp_quat.y = bounding_box.pose_stamped.pose.orientation.y;
////      tmp_quat.z = bounding_box.pose_stamped.pose.orientation.z;
////      tmp_quat.w = bounding_box.pose_stamped.pose.orientation.w;
//  tf::quaternionMsgToTF(bounding_box.pose_stamped.pose.orientation, tmp_quat);
//
//  tf::Transform base_to_bounding_box_tf;
//  base_to_bounding_box_tf.setOrigin(
//      tf::Vector3(bounding_box.pose_stamped.pose.position.x, bounding_box.pose_stamped.pose.position.y,
//                  bounding_box.pose_stamped.pose.position.z));
//  base_to_bounding_box_tf.setRotation(tmp_quat);
//
//  std::stringstream tf_name_base_to_bounding_box;
//  tf_name_base_to_bounding_box << "base_to_plane_bounding_box";
//  
//
//  tf::StampedTransform bounding_box_to_base_tf;
//  
//  bool success = false;
//  for(unsigned int i = 0; i < 1000; ++i)
//  {
//    //Publish the transform
//    tf_broadcaster->sendTransform(
//          tf::StampedTransform(base_to_bounding_box_tf, ros::Time::now(), bounding_box.pose_stamped.header.frame_id,
//                               tf_name_base_to_bounding_box.str()));
//    
//    
//    try
//    {
//      //get correct rotation: we want to rotate to have the same
//      // orientation as the base_link
//      if (tf_listener->waitForTransform(tf_name_base_to_bounding_box.str(), "/camera_depth_optical_frame", ros::Time(), ros::Duration(1.0)))
//      {
//        tf_listener->lookupTransform(tf_name_base_to_bounding_box.str(), "/camera_depth_optical_frame", ros::Time(0),
//                                     bounding_box_to_base_tf);
//      }
//      else
//      {
//        ROS_FATAL_STREAM("(1) Couldn't get the transform between /camera_depth_optical_frame and " << tf_name_base_to_bounding_box.str());
//        ROS_BREAK();
//      }
//      success = true;
//      break;
//    }
//    catch(tf::TransformException exc)
//    {
//      continue;
//    }
//  }
//  
//  if( !success )
//  {
//    ROS_FATAL_STREAM("Couldn't get the transform between /camera_depth_optical_frame and " << tf_name_base_to_bounding_box.str());
//    ROS_BREAK();
//  }

//  //We get a normal vector of the plane
//  tf::Stamped < tf::Vector3> stamped_normal(
//          tf::Vector3(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]),
//          ros::Time::now(), cloud->header.frame_id);
//
//  tf::Stamped < tf::Vector3 > stamped_normal_in_box_frame;
//  tf::Transformer aux_transformer;
//  aux_transformer.transformVector(tf_name_base_to_bounding_box.str(), stamped_normal, stamped_normal_in_box_frame);
//
//  Eigen::Vector4f z_axis_in_box_frame;
//  z_axis_in_box_frame[0] = 0.0;
//  z_axis_in_box_frame[1] = 0.0;
//  z_axis_in_box_frame[2] = 1.0;
//  z_axis_in_box_frame[3] = 0.0;
//
//  Eigen::Vector4f eigen_normal_in_box_frame;
//  eigen_normal_in_box_frame[0] = stamped_normal_in_box_frame[0];
//  eigen_normal_in_box_frame[1] = stamped_normal_in_box_frame[1];
//  eigen_normal_in_box_frame[2] = stamped_normal_in_box_frame[2];
//  eigen_normal_in_box_frame[3] = 0.0;
//pcl::IsNotDenseException
//  double angle = pcl::getAngle3D(z_axis_in_box_frame, eigen_normal_in_box_frame);
//  //Assuming that x is on the plane (which normally is almost true)
//
//  //but we don't want to translate to the base_link, just to rotate:
//  bounding_box_to_base_tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
//
//  //we want to rotate around y to make the z axis almost coinncide woth the normal
//  // so we need one more rotation, to set the z axis of the frame
//  // so we rotate  around y
//
//  tmp_quat.setRPY(0.0, angle, 0.0);
//  tf::Quaternion current_quat = bounding_box_to_base_tf.getRotation();
//
//  tmp_quat += current_quat;
//  bounding_box_to_base_tf.setRotation(tmp_quat);
//
//  std::stringstream tf_name_box_to_base_or;
//  tf_name_box_to_base_or << "box_to_base_or";
//  tf_broadcaster->sendTransform(
//      tf::StampedTransform(bounding_box_to_base_tf, ros::Time::now(), tf_name_base_to_bounding_box.str(),
//                           tf_name_box_to_base_or.str()));

  
  //PointCloud::ConstPtr transformed_cloud;
  //pcl:transformPointCloud(tf_name_box_to_base_or.str(), *cloud, *transformed_cloud, tf_listener);
  //pcl::getMinMax3D(*transformed_cloud, min_pt_, max_pt_);
  
  
  pcl::getMinMax3D(*cloud, min_pt_, max_pt_);
}

void BoxSegmentation::callback3(const pcl::ModelCoefficients::ConstPtr &coeff)
{
  //*plane_coefficients = *coeff;
}

void BoxSegmentation::read_parameters(ros::NodeHandle & nh)
{
  //Parameter reading
//  std::string base_name = nh.resolveName(this->getName(), true);
//  int param_read = 0;
//  if (nh.getParam(base_name + "/filter_max_r", param_read))
//  {
//    filter_max_r_ = static_cast<unsigned int>(param_read);
//    NODELET_INFO_STREAM("Read max r: " << filter_max_r_);
//  }
//  if (nh.getParam(base_name + "/filter_min_r", param_read))
//  {
//    filter_min_r_ = static_cast<unsigned int>(param_read);
//    NODELET_INFO_STREAM("Read min r: " << filter_min_r_);
//  }
//  if (nh.getParam(base_name + "/filter_max_g", param_read))
//  {
//    filter_max_g_ = static_cast<unsigned int>(param_read);
//    NODELET_INFO_STREAM("Read max g: " << filter_max_g_);
//  }
//  if (nh.getParam(base_name + "/filter_min_g", param_read))
//  {
//    filter_min_g_ = static_cast<unsigned int>(param_read);
//    NODELET_INFO_STREAM("Read min g: " << filter_min_g_);
//  }
//  if (nh.getParam(base_name + "/filter_max_b", param_read))
//  {
//    filter_max_b_ = static_cast<unsigned int>(param_read);
//    NODELET_INFO_STREAM("Read max b: " << filter_max_b_);
//  }
//  if (nh.getParam(base_name + "/filter_min_b", param_read))
//  {
//    filter_min_b_ = static_cast<unsigned int>(param_read);
//    NODELET_INFO_STREAM("Read min b: " << filter_min_b_);
//  }
//
//  double param_read2 = 0;
//  if (nh.getParam(base_name + "/filter_max_x", param_read2))
//  {
//    filter_max_x_ = param_read2;
//    NODELET_INFO_STREAM("Read max x: " << filter_max_x_);
//  }
//  if (nh.getParam(base_name + "/filter_min_x", param_read2))
//  {
//    filter_min_x_ = param_read2;
//    NODELET_INFO_STREAM("Read min x: " << filter_min_x_);
//  }
//  if (nh.getParam(base_name + "/filter_max_y", param_read2))
//  {
//    filter_max_y_ = param_read2;
//    NODELET_INFO_STREAM("Read max y: " << filter_max_y_);
//  }
//  if (nh.getParam(base_name + "/filter_min_y", param_read2))
//  {
//    filter_min_y_ = param_read2;
//    NODELET_INFO_STREAM("Read min y: " << filter_min_y_);
//  }
//  if (nh.getParam(base_name + "/filter_max_z", param_read2))
//  {
//    filter_max_z_ = param_read2;
//    NODELET_INFO_STREAM("Read max z: " << filter_max_z_);
//  }
//  if (nh.getParam(base_name + "/filter_min_z", param_read2))
//  {
//    filter_min_z_ = param_read2;
//    NODELET_INFO_STREAM("Read min z: " << filter_min_z_);
//  }
}
}

/* For the emacs weenies in the crowd.
 Local Variables:
 c-basic-offset: 2
 End:
 */

