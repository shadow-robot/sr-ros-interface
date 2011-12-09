/**
 * @file   kinect_color_segmentation.cpp
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
 * @brief  Segment the point cloud.
 *
 *
 */

#include <kinect_color_segmentation/point_sequence_detection.hpp>
#include <pcl/ros/conversions.h>


namespace sr_kinect
{
  PLUGINLIB_DECLARE_CLASS(sr_kinect, PointSequenceDetection, sr_kinect::PointSequenceDetection, nodelet::Nodelet);

  PointSequenceDetection::PointSequenceDetection()
    : nodelet::Nodelet(), first_time(true), line_axis("x"), K(2)
  {}

  void PointSequenceDetection::onInit()
  {
    ros::NodeHandle & nh = getNodeHandle();
    points_sub_ = nh.subscribe<PointCloud >("cloud_in",2, &PointSequenceDetection::points_callback, this);
    normals_sub_ = nh.subscribe<PointCloudNormal >("cloud_normals_in",2, &PointSequenceDetection::normals_callback, this);
    pub_ = nh.advertise<PointCloud>(this->getName() + "/output", 1000);
    point_sequence_service_ = nh.advertiseService(this->getName() + "/segment", &PointSequenceDetection::point_sequence_srv_callback, this);
    surface_normal_service_ = nh.advertiseService(this->getName() + "/get_wall_normale", &PointSequenceDetection::surface_normal_srv_callback, this);
    
    previous_pcl = boost::shared_ptr<PointCloud>(new PointCloud() );
    output_pcl = boost::shared_ptr<PointCloud>(new PointCloud() );
    srv_output_normals = boost::shared_ptr<PointCloudNormal>(new PointCloudNormal()) ;
    srv_output_pcl = boost::shared_ptr<PointCloud>(new PointCloud() );

    read_parameters(nh);
  }

  bool PointSequenceDetection::point_sequence_srv_callback(kinect_color_segmentation::SurfaceToDremmel::Request& request, kinect_color_segmentation::SurfaceToDremmel::Response& response)
  {
    boost::mutex::scoped_lock lock (mutex_);
    
    response.points.clear();
    for(unsigned int i=0; i < srv_output_pcl->size(); i++)
    {
      geometry_msgs::Point aux_point;
      aux_point.x = srv_output_pcl->at(i).x;
      aux_point.y = srv_output_pcl->at(i).y;
      aux_point.z = srv_output_pcl->at(i).z;
      response.points.push_back(aux_point);
    }
    return true;
  }
  
  bool PointSequenceDetection::surface_normal_srv_callback(kinect_color_segmentation::WallNormale::Request& request, kinect_color_segmentation::WallNormale::Response& response)
  {
    boost::mutex::scoped_lock lock (mutex_normals_);
    
    if (srv_output_normals->size() > 0)
    {
      double norm_x = 0, norm_y = 0, norm_z = 0, norm_w = 0;
      for(unsigned int i=0; i < srv_output_normals->size(); ++i)
      {
        norm_x += srv_output_normals->at(i).data_n[0];
        norm_y += srv_output_normals->at(i).data_n[1];
        norm_z += srv_output_normals->at(i).data_n[2];
        norm_w += srv_output_normals->at(i).data_n[3];
      }
      //We return the mean the normals of the cloud
      //(all of the points are supposed to be on the same plane)
      response.normale.x = norm_x / static_cast<double>(srv_output_normals->size());
      response.normale.y = norm_y / static_cast<double>(srv_output_normals->size());
      response.normale.z = norm_z / static_cast<double>(srv_output_normals->size());
      response.normale.w = norm_w / static_cast<double>(srv_output_normals->size());
      response.frame_name = srv_output_normals->header.frame_id;
      return true;
    }
    else
    {
      NODELET_WARN_STREAM(this->getName() << ": Empty normals cloud");
      return false;
    }
  }

  void PointSequenceDetection::normals_callback(const PointCloudNormal::ConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock (mutex_normals_);
    
    if (cloud->size() > 0)
    {    
      srv_output_normals = boost::shared_ptr<PointCloudNormal>(new PointCloudNormal(*cloud)) ;
    }
    else
    {
      NODELET_WARN_STREAM(this->getName() << ": Empty normals cloud received");
    }
  }
  
  void PointSequenceDetection::points_callback(const PointCloud::ConstPtr &cloud)
  {
    boost::mutex::scoped_lock lock (mutex_);
    
    if (cloud->size() > 0)
    {
      //First of all an octree is used to detect major changes in the point cloud
      //previous_pcl is only refreshed if this happens

      // Octree resolution - side length of octree voxels
      float resolution = 64.0f;
      // Instantiate octree-based point cloud change detection class
      OctreePointCloudChangeDetector octree (resolution);
  
      // Add points from cloudA to octree
      octree.setInputCloud (cloud);
      octree.addPointsFromInputCloud ();
  
      // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
      octree.switchBuffers ();
  
      if(first_time)
      {
        previous_pcl = boost::shared_ptr<PointCloud>(new PointCloud(*cloud) );
        first_time = false;
      }
  
      // Add points from cloudB to octree
      octree.setInputCloud (previous_pcl);
      octree.addPointsFromInputCloud ();
  
      std::vector<int> newPointIdxVector;
  
      // Get vector of point indices from octree voxels which did not exist in previous buffer
      octree.getPointIndicesFromNewVoxels (newPointIdxVector);
  
      if(newPointIdxVector.size() > 0) //If there are significant changes
      {
        //We'll work with the new cloud
        previous_pcl = boost::shared_ptr<PointCloud>(new PointCloud(*cloud) );
      }
  
  
      //Apply the ordering algorithm to previous_pcl cloud
      //we use a kdTreeFLANN object to find the K nearest points from a starting point
      //Ths points are added in order to an output cloud and removed from the input aux_pcl cloud
      //the method is applied recursively until all the points have been processed
  
      boost::shared_ptr<PointCloud> aux_pcl = boost::shared_ptr<PointCloud>(new PointCloud(*previous_pcl) );
  
      KdTreeFLANN kdtree;
  
      kdtree.setInputCloud (aux_pcl);
      unsigned int search_point_index = find_search_point(aux_pcl);
      pcl::PointXYZRGB searchPoint = aux_pcl->at(search_point_index);
  
  
  
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);
  
      // Create the filtering object
      ExtractIndices extract;
      pcl::PointIndices::Ptr used_points (new pcl::PointIndices ());
      output_pcl->clear();
  
      while(aux_pcl->size() > 0)
      {
        kdtree.setInputCloud (aux_pcl);
        // K nearest neighbor search
        pointIdxNKNSearch.resize(K);
        pointNKNSquaredDistance.resize(K);
  
        if ((aux_pcl->size() > 1)
            && ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
          )
        {
          used_points->indices.clear();
  
          unsigned int found_points_size = pointIdxNKNSearch.size ();
  
          if(pointIdxNKNSearch.size () > aux_pcl->size())
          {
            found_points_size = aux_pcl->size();
          }
  
          searchPoint = aux_pcl->at(pointIdxNKNSearch[found_points_size - 1]);
  
          for (size_t i = 0; i < (found_points_size - 1); ++i)
          {
            output_pcl->push_back(aux_pcl->at(pointIdxNKNSearch[i]));
            //save the current searchpoint index to delete the point
            used_points->indices.push_back(pointIdxNKNSearch[i]);
          }
  
          // Extract the used_points
          extract.setInputCloud (aux_pcl);
          extract.setIndices (used_points);
          extract.setNegative (true);
          extract.filter (*aux_pcl);
        }
        else
        {
          //save the current searchpoint
          output_pcl->push_back(aux_pcl->at(find_point_index(aux_pcl, searchPoint)));
          break;
        }
  
      }
  
      //only to visualize
      for (unsigned int j = 0; j < output_pcl->size (); ++j)
      {
        output_pcl->at(j).r = static_cast<unsigned int>(256.0 * (static_cast<double>(j)/static_cast<double>(output_pcl->size())));
        output_pcl->at(j).g = 0;
        output_pcl->at(j).b = 0;
      }
  
      output_pcl->header = aux_pcl->header;
      pub_.publish(output_pcl);
  
      //save the result for the server
      srv_output_pcl = boost::shared_ptr<PointCloud>(new PointCloud(*output_pcl) );
    }
    else
    {
      NODELET_WARN_STREAM(this->getName() << ": Empty point cloud received");
    }

  }

  void PointSequenceDetection::read_parameters(ros::NodeHandle & nh)
  {
    //Parameter reading
    std::string base_name = nh.resolveName(this->getName(), true);
    int param_read = 0;
    if (nh.getParam(base_name + "/K", param_read))
    {
      K = param_read;
      NODELET_INFO_STREAM("Read K: " << K);
    }
//    std::string param_read_2 = "";
//    if (nh.getParam(base_name + "/axis", param_read_2))
//    {
//      line_axis = param_read;
//      NODELET_INFO_STREAM("Read axis: " << line_axis);
//    }
  }

  unsigned int PointSequenceDetection::find_search_point(boost::shared_ptr<PointCloud> cloud)
  {
    //TODO find a better method to detect extreme values
    double extreme_value = -1000000.0;
    unsigned int extreme_index = 0;

    for(unsigned int i=0; i < cloud->size(); ++i)
    {
      if(line_axis == "x")
      {
        if(cloud->at(i).x > extreme_value)
        {
          extreme_value = cloud->at(i).x;
          extreme_index = i;
        }
      }
      else if(line_axis == "y")
      {
        if(cloud->at(i).y > extreme_value)
        {
          extreme_value = cloud->at(i).y;
          extreme_index = i;
        }
      }
      else if(line_axis == "z")
      {
        if(cloud->at(i).z > extreme_value)
        {
          extreme_value = cloud->at(i).z;
          extreme_index = i;
        }
      }
    }
    return extreme_index;
  }

  unsigned int PointSequenceDetection::find_point_index(boost::shared_ptr<PointCloud> cloud, pcl::PointXYZRGB searchPoint)
  {
    for(unsigned int i=0; i < cloud->size(); ++i)
    {
      if((cloud->at(i).x == searchPoint.x)
         &&(cloud->at(i).y == searchPoint.y)
         &&(cloud->at(i).z == searchPoint.z)
        )
      {
        return i;
      }
    }
    return 0;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


