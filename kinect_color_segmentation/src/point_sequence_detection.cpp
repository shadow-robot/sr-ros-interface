/**
 * @file   kinect_color_segmentation.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
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


namespace sr_kinect
{
  PLUGINLIB_DECLARE_CLASS(sr_kinect, PointSequenceDetection, sr_kinect::PointSequenceDetection, nodelet::Nodelet);

  PointSequenceDetection::PointSequenceDetection()
    : nodelet::Nodelet(), first_time(true), line_axis("x"), filter_max_r_(255), filter_min_r_(0), filter_max_g_(255), filter_min_g_(0), filter_max_b_(255), filter_min_b_(0),
      filter_max_x_(1000.0), filter_min_x_(-1000.0), filter_max_y_(1000.0), filter_min_y_(-1000.0), filter_max_z_(1000.0), filter_min_z_(-1000.0)
  {}

  void PointSequenceDetection::onInit()
  {
    ros::NodeHandle & nh = getNodeHandle();
    sub_ = nh.subscribe<PointCloud >("cloud_in",2, &PointSequenceDetection::callback, this);
    pub_ = nh.advertise<PointCloud>(this->getName() + "/output", 1000);
    previous_pcl = boost::shared_ptr<PointCloud>(new PointCloud() );
    segmented_pcl = boost::shared_ptr<PointCloud>(new PointCloud() );
    
    read_parameters(nh);
  }

  void PointSequenceDetection::callback(const PointCloud::ConstPtr &cloud)
  {
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
	      pub_.publish(previous_pcl);


//	      
//	      
//	      OctreePointCloudSearch search_octree (resolution);
//
//	        search_octree.setInputCloud (previous_pcl);
//	        search_octree.addPointsFromInputCloud ();
//	        //Find search point
//	        pcl::PointXYZRGB searchPoint = find_search_point(previous_pcl);
//
//	      
//	        
//	        while (previous_pcl->points.size()
//	      // K nearest neighbor search
//
//	        int K = 10;
//
//	        std::vector<int> pointIdxNKNSearch;
//	        std::vector<float> pointNKNSquaredDistance;
//
//	        std::cout << "K nearest neighbor search at (" << searchPoint.x 
//	                  << " " << searchPoint.y 
//	                  << " " << searchPoint.z
//	                  << ") with K=" << K << std::endl;
//
//	        if (search_octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//	        {
//	          for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
//	            std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
//	                      << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
//	                      << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
//	                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
//	        }
//	      
	      
	      
	      
	      
	      
	      
	      
	      
	      
	      
	  
	  
//    segmented_pcl->clear();
//
//    for(unsigned int i=0; i < cloud->width; ++i)
//    {
//      for(unsigned int j=0; j < cloud->height; ++j)
//      {
//		if((( cloud->at(i,j).r) > filter_min_r_)
//          && (( cloud->at(i,j).r) < filter_max_r_)
//          &&(( cloud->at(i,j).g) > filter_min_g_)
//          &&(( cloud->at(i,j).g) < filter_max_g_)
//          &&(( cloud->at(i,j).b) > filter_min_b_)
//          &&(( cloud->at(i,j).b) < filter_max_b_)  
//          &&(( cloud->at(i,j).x) > filter_min_x_)
//          &&(( cloud->at(i,j).x) < filter_max_x_)
//          &&(( cloud->at(i,j).y) > filter_min_y_)
//          &&(( cloud->at(i,j).y) < filter_max_y_)
//          &&(( cloud->at(i,j).z) > filter_min_z_)
//          &&(( cloud->at(i,j).z) < filter_max_z_)  
//          )
//          segmented_pcl->push_back( cloud->at(i,j) );
//      }
//    }
//    segmented_pcl->header.frame_id = cloud->header.frame_id;
//    
//    pub_.publish(segmented_pcl);
  }  

  void PointSequenceDetection::read_parameters(ros::NodeHandle & nh)
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
  	
  	double param_read2 = 0;
	if (nh.getParam(base_name + "/filter_max_x", param_read2))
	{
		filter_max_x_ = param_read2;
		NODELET_INFO_STREAM("Read max x: " << filter_max_x_);
	}
	if (nh.getParam(base_name + "/filter_min_x", param_read2))
	{
		filter_min_x_ = param_read2;
		NODELET_INFO_STREAM("Read min x: " << filter_min_x_);
	}
	if (nh.getParam(base_name + "/filter_max_y", param_read2))
	{
		filter_max_y_ = param_read2;
		NODELET_INFO_STREAM("Read max y: " << filter_max_y_);
	}
	if (nh.getParam(base_name + "/filter_min_y", param_read2))
	{
		filter_min_y_ = param_read2;
		NODELET_INFO_STREAM("Read min y: " << filter_min_y_);
	}
	if (nh.getParam(base_name + "/filter_max_z", param_read2))
	{
		filter_max_z_ = param_read2;
		NODELET_INFO_STREAM("Read max z: " << filter_max_z_);
	}
	if (nh.getParam(base_name + "/filter_min_z", param_read2))
	{
		filter_min_z_ = param_read2;
		NODELET_INFO_STREAM("Read min z: " << filter_min_z_);
	}
  }
  
  pcl::PointXYZRGB PointSequenceDetection::find_search_point(boost::shared_ptr<PointCloud> cloud)
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
  	return cloud->at(extreme_index);
  }
}




/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


