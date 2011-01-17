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

#include "tabletop_collision_map_processing/collision_map_interface.h"

#include <tf/transform_datatypes.h>

#include <std_srvs/Empty.h>

#include <mapping_msgs/CollisionObject.h>
#include <mapping_msgs/AttachedCollisionObject.h>

#include <object_manipulation_msgs/FindClusterBoundingBox.h>

#include <household_objects_database_msgs/DatabaseModelPose.h>
#include <household_objects_database_msgs/GetModelMesh.h>

namespace tabletop_collision_map_processing {

static const std::string RESET_COLLISION_MAP_NAME = "collision_map_self_occ_node/reset";
static const std::string MAKE_STATIC_COLLISION_MAP_ACTION_NAME = "make_static_collision_map";
static const std::string CLUSTER_BOUNDING_BOX_NAME = "find_cluster_bounding_box";

CollisionMapInterface::CollisionMapInterface() : 
  root_nh_(""),
  priv_nh_("~"),
  collision_object_current_id_(0),
  make_static_collision_map_client_(root_nh_, MAKE_STATIC_COLLISION_MAP_ACTION_NAME, true)
{
  collision_object_pub_ = root_nh_.advertise<mapping_msgs::CollisionObject>("collision_object", 10);

  attached_object_pub_ = root_nh_.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  
  while ( !ros::service::waitForService(RESET_COLLISION_MAP_NAME, ros::Duration(2.0)) && root_nh_.ok() ) 
  {
    ROS_INFO("Waiting for reset collision map service to come up");
  }
  collision_map_reset_client_ = root_nh_.serviceClient<std_srvs::Empty>(RESET_COLLISION_MAP_NAME, true);
  
  while ( !ros::service::waitForService(CLUSTER_BOUNDING_BOX_NAME, ros::Duration(2.0)) && root_nh_.ok() ) 
  {
    ROS_INFO("Waiting for cluster bounding box service to come up");
  }
  cluster_bounding_box_client_ = root_nh_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox>
    (CLUSTER_BOUNDING_BOX_NAME, true);

  while(!make_static_collision_map_client_.waitForServer(ros::Duration(2.0))
	&& root_nh_.ok()){
    ROS_INFO("Waiting for the make static collision map action to come up");
  }
}

void CollisionMapInterface::resetCollisionModels()
{
  mapping_msgs::CollisionObject reset_object;
  reset_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  reset_object.header.frame_id = "base_link";
  reset_object.header.stamp = ros::Time::now();
  reset_object.id = "all";
  collision_object_pub_.publish(reset_object);
  collision_object_current_id_ = 0;
}

void CollisionMapInterface::resetAttachedModels()
{
  mapping_msgs::CollisionObject reset_object;
  reset_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  reset_object.header.frame_id = "base_link";
  reset_object.header.stamp = ros::Time::now();
  reset_object.id = "all";
  mapping_msgs::AttachedCollisionObject reset_attached_objects;
  reset_attached_objects.object.header.frame_id = "base_link";
  reset_attached_objects.object.header.stamp = ros::Time::now();
  reset_attached_objects.link_name = "all";
  reset_attached_objects.object = reset_object;
  attached_object_pub_.publish(reset_attached_objects);
}

void CollisionMapInterface::resetStaticMap() 
{
  std_srvs::Empty srv; 
  if (!collision_map_reset_client_.call(srv))
  {
    ROS_ERROR("Collision map reset call failed");
    throw CollisionMapException("reset failed");
  }
}

std::string CollisionMapInterface::getNextObjectName()
{
  std::ostringstream iss;
  iss << collision_object_current_id_++;
  return "graspable_object_" + iss.str();
}

void CollisionMapInterface::takeStaticMap() 
{  
  collision_environment_msgs::MakeStaticCollisionMapGoal static_map_goal;
  static_map_goal.cloud_source = "camera/depth/points";
  static_map_goal.number_of_clouds = 2;
  
  make_static_collision_map_client_.sendGoal(static_map_goal);

  if ( !make_static_collision_map_client_.waitForResult(ros::Duration(30.0)) )
  {
    ROS_ERROR("Collision map was not formed in allowed time");
    throw CollisionMapException("static make was not formed in allowed time");
  }

  if(make_static_collision_map_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
  {
    ROS_ERROR("Some non-success state was reached for static collision map.  Proceed with caution");
    throw CollisionMapException(" static collision map failed");
  } 
}

void CollisionMapInterface::processCollisionGeometryForTable(const tabletop_object_detector::Table &table,
							     std::string table_collision_name) 
{
  mapping_msgs::CollisionObject table_object;
  table_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  table_object.header.frame_id = table.pose.header.frame_id;
  table_object.header.stamp = ros::Time::now();
  table_object.shapes.resize(1);
  table_object.shapes[0].type = geometric_shapes_msgs::Shape::BOX;
  table_object.shapes[0].dimensions.resize(3);
  table_object.shapes[0].dimensions[0] = fabs(table.x_max-table.x_min);
  table_object.shapes[0].dimensions[1] = fabs(table.y_max-table.y_min);
  table_object.shapes[0].dimensions[2] = 0.01;

  //set the origin of the table object in the middle of the table
  tf::Transform table_trans;
  tf::poseMsgToTF(table.pose.pose, table_trans);
  tf::Transform table_translation;
  table_translation.setIdentity();
  table_translation.setOrigin( btVector3( (table.x_min + table.x_max)/2.0, (table.y_min + table.y_max)/2.0, 0.0) );
  table_trans = table_trans * table_translation;
  table_object.poses.resize(1);
  tf::poseTFToMsg(table_trans, table_object.poses[0]);

  table_object.id = table_collision_name;
  collision_object_pub_.publish(table_object);
}

void CollisionMapInterface::processCollisionGeometryForObject
  (const household_objects_database_msgs::DatabaseModelPose &model_pose,
   std::string &collision_name)
{
  mapping_msgs::CollisionObject collision_object;
  collision_object.shapes.resize(1);

  if (!getMeshFromDatabasePose(model_pose, collision_object.shapes[0]))
  {
    throw CollisionMapException("Loading mesh for database object failed");
  }
  collision_object.header.frame_id = model_pose.pose.header.frame_id;
  collision_object.header.stamp = ros::Time::now();
  collision_object.poses.push_back(model_pose.pose.pose);
  
  collision_object.shapes[0].type = geometric_shapes_msgs::Shape::MESH;
  collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  collision_name = getNextObjectName();
  collision_object.id = collision_name;
  collision_object_pub_.publish(collision_object);
}

void 
CollisionMapInterface::processCollisionGeometryForBoundingBox(const object_manipulation_msgs::ClusterBoundingBox &box, 
							      std::string &collision_name)
{
  ROS_INFO("Adding bounding box with dimensions %f %f %f to collision map", 
	   box.dimensions.x, box.dimensions.y, box.dimensions.z);

  mapping_msgs::CollisionObject collision_object;
  collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;

  collision_name = getNextObjectName();
  collision_object.id = collision_name;

  collision_object.header.frame_id = box.pose_stamped.header.frame_id;
  collision_object.header.stamp = ros::Time::now();
  
  geometric_shapes_msgs::Shape shape;
  shape.type = geometric_shapes_msgs::Shape::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[0] = box.dimensions.x;
  shape.dimensions[1] = box.dimensions.y;
  shape.dimensions[2] = box.dimensions.z;
  collision_object.shapes.push_back(shape);
  collision_object.poses.push_back(box.pose_stamped.pose);

  collision_object_pub_.publish(collision_object);
}

void CollisionMapInterface::processCollisionGeometryForCluster(const sensor_msgs::PointCloud &cluster,
							       std::string &collision_name)
{
  ROS_INFO("Adding cluster with %u points to collision map", (unsigned int)cluster.points.size()); 

  mapping_msgs::CollisionObject many_boxes;
  many_boxes.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  many_boxes.header = cluster.header;
  many_boxes.header.stamp = ros::Time::now();
  unsigned int num_to_use = (unsigned int)(cluster.points.size()/100.0);
  many_boxes.shapes.resize(num_to_use);
  many_boxes.poses.resize(num_to_use);
  for(unsigned int i = 0; i < num_to_use; i++) {
    geometric_shapes_msgs::Shape shape;
    shape.type = geometric_shapes_msgs::Shape::BOX;
    shape.dimensions.resize(3);
    shape.dimensions[0] = .005;
    shape.dimensions[1] = .005;
    shape.dimensions[2] = .005;
    many_boxes.shapes[i]=shape;
    geometry_msgs::Pose pose;
    pose.position.x = cluster.points[i*100].x;
    pose.position.y = cluster.points[i*100].y;
    pose.position.z = cluster.points[i*100].z;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    many_boxes.poses[i] = pose;
  }

  collision_name = getNextObjectName();
  //use this name for the collision map
  many_boxes.id = collision_name;
  collision_object_pub_.publish(many_boxes);
}

bool CollisionMapInterface::getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,
						    geometric_shapes_msgs::Shape& mesh) 
{
  static bool service_initialized = false;
  if (!service_initialized)
  {    
    std::string get_model_mesh_srv_name;
    priv_nh_.param<std::string>("get_model_mesh_srv", get_model_mesh_srv_name, "get_model_mesh_srv");
    while ( !ros::service::waitForService(get_model_mesh_srv_name, ros::Duration(2.0)) && root_nh_.ok() ) 
    {
      ROS_INFO("Waiting for %s service to come up", get_model_mesh_srv_name.c_str());
    }
    if (!root_nh_.ok()) exit(0);
    get_model_mesh_srv_ = root_nh_.serviceClient<household_objects_database_msgs::GetModelMesh>
      (get_model_mesh_srv_name, true);
    service_initialized = true;
  }
  household_objects_database_msgs::GetModelMesh get_mesh;
  get_mesh.request.model_id = model_pose.model_id;
  if ( !get_model_mesh_srv_.call(get_mesh) || 
       get_mesh.response.return_code.code != get_mesh.response.return_code.SUCCESS )
  {
    return false;
  }
  mesh = get_mesh.response.mesh;
  return true;
}

void CollisionMapInterface::getClusterBoundingBox(const sensor_msgs::PointCloud &cluster,
						  geometry_msgs::PoseStamped &pose_stamped, 
						  geometry_msgs::Vector3 &dimensions)
{
  object_manipulation_msgs::FindClusterBoundingBox srv;
  srv.request.cluster = cluster;
  if (!cluster_bounding_box_client_.call(srv.request, srv.response))
  {
    ROS_ERROR("Failed to call cluster bounding box client");
    throw CollisionMapException("Failed to call cluster bounding box client");
  }
  pose_stamped = srv.response.pose;
  dimensions = srv.response.box_dims;
  if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0)
  {
    ROS_ERROR("Cluster bounding box client returned an error (0.0 bounding box)");
    throw CollisionMapException("Bounding box computation failed");  
  }
}

/*! Assumes the bounding box is in the same frame as the table */
bool CollisionMapInterface::extendBoundingBoxZToTable(const tabletop_object_detector::Table &table,
                                                      geometry_msgs::PoseStamped &pose_stamped, 
                                                      geometry_msgs::Vector3 &dimensions)
{
  if (table.pose.header.frame_id != pose_stamped.header.frame_id)
  {
    ROS_INFO("cannot extend bbox to table, they are not in the same frame");
    return false;
  }

  //get the distance from the box to the table
  btVector3 table_location(table.pose.pose.position.x,
                           table.pose.pose.position.y,
                           table.pose.pose.position.z);
  btVector3 bbox_location(pose_stamped.pose.position.x, 
                          pose_stamped.pose.position.y, 
                          pose_stamped.pose.position.z);
  btVector3 table_to_bbox = bbox_location - table_location;
  btTransform table_trans;
  tf::poseMsgToTF(table.pose.pose, table_trans);
  btVector3 table_z = table_trans.getBasis().getColumn(2);
  ROS_INFO("Table z: %f %f %f in frame %s", table_z.getX(), table_z.getY(), table_z.getZ(), table.pose.header.frame_id.c_str());
  double box_to_table_distance = table_to_bbox.dot(table_z);
  ROS_INFO("Table distance: %f", box_to_table_distance);

  //now we actually make the assumptions that the z axes are close to each other
  //as doing an actual computation was just too hard
  btTransform bbox_trans;
  tf::poseMsgToTF(pose_stamped.pose, bbox_trans);
  btVector3 bbox_z = bbox_trans.getBasis().getColumn(2);
  if (fabs(bbox_z.dot(table_z)) < 0.9848) //10 degrees
  { 
    ROS_INFO("cannot extend bbox to table; z axes do not coincide");
    return false;
  }
  
  double z_to_table = box_to_table_distance;
  ROS_INFO("z_to_table: %f", z_to_table);
  if ( z_to_table < 0.005 || z_to_table > 0.3)
  {
    ROS_INFO("cannot extend bbox to table; getting z equal to %f", z_to_table);
    return false;
  }
  ROS_INFO("Old z: %f", dimensions.z);
  double new_z = dimensions.z/2.0 + z_to_table;
  ROS_INFO("New z: %f", new_z);
  pose_stamped.pose.position.z = pose_stamped.pose.position.z + (dimensions.z/2.0) - (new_z/2.0);
  dimensions.z = new_z;
  return true;
}


} //namespace tabletop_collision_map_processing
