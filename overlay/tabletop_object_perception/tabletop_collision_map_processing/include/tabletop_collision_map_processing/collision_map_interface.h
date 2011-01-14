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

// Author(s): Gil Jones and Matei Ciocarlie

#ifndef _COLLISION_MAP_INTERFACE_H_
#define _COLLISION_MAP_INTERFACE_H_

#include <stdexcept>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <collision_environment_msgs/MakeStaticCollisionMapAction.h>

#include <household_objects_database_msgs/DatabaseModelPose.h>

#include <sensor_msgs/PointCloud.h>

#include <geometric_shapes_msgs/Shape.h>

#include <tabletop_object_detector/Table.h>
#include <tabletop_object_detector/TabletopDetectionResult.h>

#include <object_manipulation_msgs/GraspableObject.h>
#include <object_manipulation_msgs/ClusterBoundingBox.h>

namespace tabletop_collision_map_processing {

//! General base class for all exceptions originating in the collision map interface
class CollisionMapException : public std::runtime_error
{ 
 public:
 CollisionMapException(const std::string error) : std::runtime_error("collision map: "+error) {};
};

//! Provides the collision map related services needed by object grasping
class CollisionMapInterface
{
 private:
  //! The root node handle
  ros::NodeHandle root_nh_;

  //! The private node handle
  ros::NodeHandle priv_nh_;

  //! Publisher for objects in map
  ros::Publisher collision_object_pub_;

  //! Publisher for attached objects
  ros::Publisher attached_object_pub_;

  //! Client for resetting the dynamic collision map
  ros::ServiceClient collision_map_reset_client_;
  
  //! Client for computing the outlier-resistant bounding box of a cluster
  ros::ServiceClient cluster_bounding_box_client_;

  //! Client for getting the mesh for a database object
  ros::ServiceClient get_model_mesh_srv_;

  //! Counts the number of objects added to the map since the last reset
  int collision_object_current_id_;

  //! Action client for static map
  actionlib::SimpleActionClient<collision_environment_msgs::MakeStaticCollisionMapAction> 
    make_static_collision_map_client_;

  //! Produces an object mesh from a database object
  bool getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,
                               geometric_shapes_msgs::Shape& mesh);

  //! Creates a name for an object based on collision_object_current_id_, which is then incremented
  std::string getNextObjectName();

 public:
  //! Connects and waits for all needed services and action servers
  CollisionMapInterface();

  //! Sends table info into the collision map
  void processCollisionGeometryForTable(const tabletop_object_detector::Table &table,
					std::string table_collision_name);

  //! Sends collision geometry for object into the collision map
  void processCollisionGeometryForObject(const household_objects_database_msgs::DatabaseModelPose &model_pose, 
					 std::string &collision_name);
  
  //! Sends collision geometry for a point cloud into the collision map
  void processCollisionGeometryForCluster(const sensor_msgs::PointCloud &cluster, 
					  std::string &collision_name);

  //! Sends collision geometry for a bounding box into the collision map
  void processCollisionGeometryForBoundingBox(const object_manipulation_msgs::ClusterBoundingBox &box, 
					      std::string &collision_name);

  //! Removes all collision models from the map
  void resetCollisionModels();

  //! Removes all the models attached to the robot in the collision map
  void resetAttachedModels();

  //! Resets the static collision map
  void resetStaticMap();

  //! Sets the laser to slow and creates a static collision map
  void takeStaticMap();

  //! Computes the outlier-resistant bounding box for a cluster
  void getClusterBoundingBox(const sensor_msgs::PointCloud &cluster,
			     geometry_msgs::PoseStamped &pose_stamped, 
			     geometry_msgs::Vector3 &dimensions);

  //! Extends a bounding box along its Z dimension until it touches the table
  bool extendBoundingBoxZToTable(const tabletop_object_detector::Table &table,
                                 geometry_msgs::PoseStamped &pose_stamped, 
                                 geometry_msgs::Vector3 &dimensions);
};

} //namespace tabletop_collision_map_perception

#endif
