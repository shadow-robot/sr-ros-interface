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

#include <kinect_color_segmentation/kinect_color_segmentation.hpp>

namespace sr_kinect
{
  PLUGINLIB_DECLARE_CLASS(sr_kinect, KinectColorSegmentation, sr_kinect::KinectColorSegmentation, nodelet::Nodelet);

  KinectColorSegmentation::KinectColorSegmentation()
    : nodelet::Nodelet()
  {}

  void KinectColorSegmentation::onInit()
  {
    viewer = new pcl_visualization::CloudViewer("Cloud 3D");
    ros::NodeHandle & nh = getNodeHandle();
    sub_ = nh.subscribe<PointCloud >("cloud_in",2, &KinectColorSegmentation::callback, this);
    pub_ = nh.advertise<PointCloud>("seg_output", 1000);
    segmented_pcl = boost::shared_ptr<PointCloud>(new PointCloud() );
  }

  void KinectColorSegmentation::callback(const PointCloud::ConstPtr &cloud)
  {
    segmented_pcl->clear();

    for(unsigned int i=0; i < cloud->width; ++i)
    {
      for(unsigned int j=0; j < cloud->height; ++j)
      {
//        if((( cloud->at(i,j).rgb & 0xFF0000) > 0x700000)
//          && (( cloud->at(i,j).rgb & 0xFF0000) < 0xF00000)
//          &&(( cloud->at(i,j).rgb & 0x00FF00) > 0x007000)
//          &&(( cloud->at(i,j).rgb & 0x00FF00) < 0x00F000)
//          &&(( cloud->at(i,j).rgb & 0x0000FF) > 0x000070)
//          &&(( cloud->at(i,j).rgb & 0x0000FF) < 0x0000F0)        		
//          )
		if((( cloud->at(i,j).r) > 0x70)
				  && (( cloud->at(i,j).r) < 0xC0)
				  &&(( cloud->at(i,j).g) > 0x70)
				  &&(( cloud->at(i,j).g) < 0xC0)
				  &&(( cloud->at(i,j).b) > 0x70)
				  &&(( cloud->at(i,j).b) < 0xC0)        		
				  )
          segmented_pcl->push_back( cloud->at(i,j) );
      }
    }
    segmented_pcl->header.frame_id = cloud->header.frame_id;
    
    pub_.publish(segmented_pcl);
    
    ros::spinOnce();

    if(!viewer->wasStopped())
    {
      viewer->showCloud( *(segmented_pcl.get()) );
    }
  }

  KinectColorSegmentation::~KinectColorSegmentation()
  {
    if(viewer != NULL)
      delete viewer;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


