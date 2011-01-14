
/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author(s): Matei Ciocarlie

#include <ros/ros.h>

#include <string>

#include "tabletop_object_detector/TabletopDetection.h"

/*! Simply pings the ObjectDetection service provided by the tabletop node
  and prints out the result.
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_tabletop_node");
  ros::NodeHandle nh;
  std::string service_name("/object_detection");
  tabletop_object_detector::TabletopDetection srv;
  srv.request.return_clusters = true;
  srv.request.return_models = true;
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) )
  {
    ROS_INFO("Waiting for service...");
  }

  if (ros::service::call(service_name, srv))
  {
    if (srv.response.detection.result == srv.response.detection.NO_CLOUD_RECEIVED)
      {
	ROS_INFO("Detector reports no cloud received");
      }
    else if (srv.response.detection.result == srv.response.detection.NO_TABLE)
      {
	ROS_INFO("Detector reports no table detected");
      }
    else if (srv.response.detection.result == srv.response.detection.SUCCESS)
      {
	if (srv.response.detection.models.empty())
	  {
	    ROS_INFO("Table detected, but no models found");
	  }
	else
	  {
	    ROS_INFO("Detected %d models:",(int)srv.response.detection.models.size());
	    for (size_t i=0; i<srv.response.detection.models.size(); i++)
	      {
		ROS_INFO("  Model id %d detected",srv.response.detection.models[i].model_id);
	      }
	    ROS_INFO("Detected %d clusters:", (int)srv.response.detection.clusters.size());
	    for (size_t i=0; i<srv.response.detection.clusters.size(); i++)
	      {
		if (srv.response.detection.cluster_model_indices.at(i) < 0)
		  {
		    ROS_INFO("  Unidentifiable cluster");
		  }
		else
		  {
		    ROS_INFO("  Cluster corresponding to model %d", srv.response.detection.cluster_model_indices.at(i));
		  }
	      }
	  }
      }
    else
      {
	ROS_INFO("Detector reports some other error");
      }
  }
  else
  {
    ROS_ERROR("Service call failed");
  }
  return true;
}
