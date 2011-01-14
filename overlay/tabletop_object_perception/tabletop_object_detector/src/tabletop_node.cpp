
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
  
// Author(s): Marius Muja and Matei Ciocarlie

#include <string>

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <household_objects_database_msgs/DatabaseModelPose.h>
#include <household_objects_database_msgs/DatabaseModelPoseArray.h>
#include <household_objects_database_msgs/GetModelMesh.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include "tabletop_object_detector/exhaustive_fit_detector.h"
#include "tabletop_object_detector/marker_generator.h"
#include "tabletop_object_detector/iterative_distance_fitter.h"
#include "tabletop_object_detector/Table.h"
#include "tabletop_object_detector/TabletopDetectionResult.h"
#include "tabletop_object_detector/TabletopDetection.h"

namespace tabletop_object_detector {

class TabletopNode 
{
  typedef pcl::PointXYZ    Point;
  typedef pcl::KdTree<Point>::Ptr KdTreePtr;
  
private:
  //! The node handle
  ros::NodeHandle nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;
  //! Listener for incoming new-style point clouds
  ros::Subscriber cloud_new_sub_;
  //! Publisher for markers
  ros::Publisher marker_pub_;
  //! Service server for object detection
  ros::ServiceServer object_detection_srv_;

  //! Service client for getting a mesh from the database
  ros::ServiceClient get_model_mesh_srv_;

  //! The threshold for deciding good vs. bad fits; hard-coded in for now
  double quality_threshold_;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;

  //! The current marker being published
  int current_marker_id_;

  //! Provides exclusion between service callback and message callbacks
  boost::mutex callback_mutex_;

  //! The instance of the detector used for all detecting tasks
  ExhaustiveFitDetector<IterativeTranslationFitter> detector_;

  //! Whether to use the database of known models or not
  bool use_database_;
  //! Whether to use a reduced model set from the database
  std::string model_set_;

  //! Whether to perform a merge step based on model fit results
  bool perform_fit_merge_;
  //! The threshold for merging two models that were fit very close to each other
  double fit_merge_threshold_;
  //! Min number of inliers for reliable plane detection
  int inlier_threshold_;
  //! Size of downsampling grid before performing plane detection
  double plane_detection_voxel_size_;
  //! Size of downsampling grid before performing clustering
  double clustering_voxel_size_;
  //! Filtering of original point cloud along the z axis
  double z_filter_min_, z_filter_max_;
  //! Filtering of point cloud in table frame after table detection
  double table_z_filter_min_, table_z_filter_max_;
  //! Min distance between two clusters
  double cluster_distance_;
  //! Min number of points for a cluster
  int min_cluster_size_;
  //! Clouds are transformed into this frame before processing; leave empty if clouds
  //! are to be processed in their original frame
  std::string processing_frame_;
  //! Positive or negative z is closer to the "up" direction in the processing frame?
  double up_direction_;

  //! Whether to publish a line marker around the fit table 
  bool publish_table_marker_;
  //! Whether to publish a colored point cloud for each marker
  bool publish_cluster_markers_;
  //! Whether to publish an outline of the fit model at the fit location for fits above quality threshold
  bool publish_good_fit_markers_;
  //! Whether to publish an outline of the fit model at the fit location for fits below quality threshold
  bool publish_bad_fit_markers_;

  //! A tf transform listener
  tf::TransformListener listener_;

  //------------------ Callbacks -------------------

  //! Callback for incoming clouds;
  void cloudCallbackNew(const sensor_msgs::PointCloud2::ConstPtr& cloud);

  //! Callback for service call
  bool serviceCallback(TabletopDetection::Request &request, TabletopDetection::Response &response);

  //------------------ Individual processing steps -------

  //! Performs object detection on the given clusters, can also merge clusters based on detection result
  template <class PointCloudType>
  void objectDetection(roslib::Header cloud_header, std::vector<PointCloudType> &clusters,
		       const tf::Transform &table_plane_trans, TabletopDetectionResult &detection_message);

  //! Converts raw table detection results into a Table message type
  template <class PointCloudType>
  Table getTable(roslib::Header cloud_header, const tf::Transform &table_plane_trans,
		 const PointCloudType &table_points);

  //! Publishes rviz markers for the given tabletop clusters
  template <class PointCloudType>
  void publishClusterMarkers(const std::vector<PointCloudType> &clusters, 
			     roslib::Header cloud_header, const tf::Transform &table_plane_trans);

  //! Transforms the given clusters from the table frame into the original cloud frame
  bool transformClustersToCloudFrame(std::vector<sensor_msgs::PointCloud> &clusters,
				     roslib::Header cloud_header,
				     tf::Transform table_plane_trans);

  //------------------- Complete processing -----

  //! Complete processing for new style point cloud
  void processCloud(const sensor_msgs::PointCloud2 &cloud,
		    bool return_clusters, bool return_fits,
		    TabletopDetectionResult &detection_message);

  //-------------------- Misc -------------------

  //! Helper function that returns the distance along the plane between two fit models
  double fitDistance(const ModelFitInfo &m1, const ModelFitInfo &m2);

  //! Helper function that returns the distance along the plane between a fit model and a cluster
  template <class PointCloudType>
  double fitClusterDistance(const ModelFitInfo &m, const PointCloudType &cluster);

  //! Clears old published markers and remembers the current number of published markers
  void clearOldMarkers(std::string frame_id);



public:
  //! Subscribes to and advertises topics; initializes fitter and marker publication flags
  /*! Also attempts to connect to database */
  TabletopNode(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
    num_markers_published_ = 1;
    current_marker_id_ = 1;

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("markers_out"), 10);

    priv_nh_.param<bool>("use_database", use_database_, false);
    if (use_database_)
    {
      std::string get_model_mesh_srv_name;
      priv_nh_.param<std::string>("get_model_mesh_srv", get_model_mesh_srv_name, "get_model_mesh_srv");
      while ( !ros::service::waitForService(get_model_mesh_srv_name, ros::Duration(2.0)) && nh_.ok() ) 
      {
	ROS_INFO("Waiting for %s service to come up", get_model_mesh_srv_name.c_str());
      }
      if (!nh_.ok()) exit(0);
      get_model_mesh_srv_ = nh_.serviceClient<household_objects_database_msgs::GetModelMesh>
	(get_model_mesh_srv_name, true);
      //ask fitter to load models from database
      priv_nh_.param<std::string>("model_set", model_set_, "");
      detector_.loadDatabaseModels(model_set_);
    }

    object_detection_srv_ = nh_.advertiseService(nh_.resolveName("object_detection_srv"), 
                                                 &TabletopNode::serviceCallback, this);

    //initialize operational flags
    priv_nh_.param<bool>("perform_fit_merge", perform_fit_merge_, true);
    priv_nh_.param<double>("fit_merge_threshold", fit_merge_threshold_, 0.05);
    priv_nh_.param<int>("inlier_threshold", inlier_threshold_, 300);
    priv_nh_.param<double>("plane_detection_voxel_size", plane_detection_voxel_size_, 0.01);
    priv_nh_.param<double>("clustering_voxel_size", clustering_voxel_size_, 0.003);
    priv_nh_.param<double>("z_filter_min", z_filter_min_, 0.4);
    priv_nh_.param<double>("z_filter_max", z_filter_max_, 1.25);
    priv_nh_.param<double>("table_z_filter_min", table_z_filter_min_, 0.01);
    priv_nh_.param<double>("table_z_filter_max", table_z_filter_max_, 0.50);
    priv_nh_.param<double>("cluster_distance", cluster_distance_, 0.03);
    priv_nh_.param<int>("min_cluster_size", min_cluster_size_, 300);
    priv_nh_.param<double>("quality_threshold", quality_threshold_, 0.005);
    priv_nh_.param<std::string>("processing_frame", processing_frame_, "");
    priv_nh_.param<double>("up_direction", up_direction_, -1.0);
   
    //initialize marker flags
    priv_nh_.param<bool>("publish_table_marker", publish_table_marker_, true);
    priv_nh_.param<bool>("publish_cluster_markers", publish_cluster_markers_, true);
    priv_nh_.param<bool>("publish_good_fit_markers", publish_good_fit_markers_, true);
    priv_nh_.param<bool>("publish_bad_fit_markers", publish_bad_fit_markers_, false);
  }

  //! Empty stub
  ~TabletopNode() {}
};

/*! Processes the latest point cloud and gives back the resulting array of models.
 */
bool TabletopNode::serviceCallback(TabletopDetection::Request &request, TabletopDetection::Response &response)
{

  std::string topic = nh_.resolveName("cloud_new_in");
  ROS_INFO("Tabletop detection service called; waiting for a point_cloud2 on topic %s", topic.c_str());

  sensor_msgs::PointCloud2::ConstPtr recent_cloud = 
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh_, ros::Duration(3.0));
              
  if (!recent_cloud)
  {
    ROS_ERROR("Tabletop object detector: no point_cloud2 has been received");
    response.detection.result = response.detection.NO_CLOUD_RECEIVED;
    return true;
  }

  ROS_INFO("Point cloud received; processing");
  if (!processing_frame_.empty())
  {
    //convert cloud to base link frame
    sensor_msgs::PointCloud old_cloud;  
    sensor_msgs::convertPointCloud2ToPointCloud (*recent_cloud, old_cloud);
    try
    {
      listener_.transformPointCloud(processing_frame_, old_cloud, old_cloud);    
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Failed to transform cloud from frame %s into frame %s", old_cloud.header.frame_id.c_str(), 
		processing_frame_.c_str());
      response.detection.result = response.detection.OTHER_ERROR;
      return true;
    }
    sensor_msgs::PointCloud2 converted_cloud;
    sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, converted_cloud);
    ROS_INFO("Input cloud converted to %s frame", processing_frame_.c_str());
    processCloud(converted_cloud, request.return_clusters, request.return_models, response.detection);
  }
  else
  {
    processCloud(*recent_cloud, request.return_clusters, request.return_models, response.detection);
  }

  return true;
}

template <class PointCloudType>
Table TabletopNode::getTable(roslib::Header cloud_header,
			     const tf::Transform &table_plane_trans, 
			     const PointCloudType &table_points)
{
  Table table;
 
  //get the extents of the table
  if (!table_points.points.empty()) 
  {
    table.x_min = table_points.points[0].x;
    table.x_max = table_points.points[0].x;
    table.y_min = table_points.points[0].y;
    table.y_max = table_points.points[0].y;
  }  
  for (size_t i=1; i<table_points.points.size(); ++i) 
  {
    if (table_points.points[i].x<table.x_min && table_points.points[i].x>-3.0) table.x_min = table_points.points[i].x;
    if (table_points.points[i].x>table.x_max && table_points.points[i].x< 3.0) table.x_max = table_points.points[i].x;
    if (table_points.points[i].y<table.y_min && table_points.points[i].y>-3.0) table.y_min = table_points.points[i].y;
    if (table_points.points[i].y>table.y_max && table_points.points[i].y< 3.0) table.y_max = table_points.points[i].y;
  }

  geometry_msgs::Pose table_pose;
  tf::poseTFToMsg(table_plane_trans, table_pose);
  table.pose.pose = table_pose;
  table.pose.header = cloud_header;
  if (publish_table_marker_)
  {
    visualization_msgs::Marker tableMarker = MarkerGenerator::getTableMarker(table.x_min, table.x_max,
									     table.y_min, table.y_max);
    tableMarker.header = cloud_header;
    tableMarker.pose = table_pose;
    tableMarker.ns = "tabletop_node";
    tableMarker.id = current_marker_id_++;
    marker_pub_.publish(tableMarker);
  }  
  return table;
}

template <class PointCloudType>
void TabletopNode::publishClusterMarkers(const std::vector<PointCloudType> &clusters, 
					 roslib::Header cloud_header, const tf::Transform &table_plane_trans)
{
  geometry_msgs::Pose table_pose;
  tf::poseTFToMsg(table_plane_trans, table_pose);
  for (size_t i=0; i<clusters.size(); i++) 
  {
    ROS_INFO("Cloud size: %d",(int)clusters[i].points.size());
    visualization_msgs::Marker cloud_marker =  MarkerGenerator::getCloudMarker(clusters[i]);
    cloud_marker.header = cloud_header;
    //set the marker in the pose of the table
    cloud_marker.pose = table_pose;
    cloud_marker.ns = "tabletop_node";
    cloud_marker.id = current_marker_id_++;
    marker_pub_.publish(cloud_marker);
  }
}

bool TabletopNode::transformClustersToCloudFrame(std::vector<sensor_msgs::PointCloud> &clusters,
						 roslib::Header cloud_header,
						 tf::Transform table_plane_trans)
{
  tf::TransformListener listener;
  ros::Time now_stamp = ros::Time::now();
  tf::StampedTransform table_pose_frame_inv(table_plane_trans.inverse(), now_stamp, 
					    "table_frame", cloud_header.frame_id);
  listener.setTransform(table_pose_frame_inv);
  bool success = true;
  for (size_t i=0; i<clusters.size(); i++)
  {
    clusters[i].header.stamp = now_stamp;
    clusters[i].header.frame_id = "table_frame";
    try 
    {
      listener.transformPointCloud(cloud_header.frame_id, clusters[i], clusters[i]);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("Failed to transform cluster from table frame back into cloud frame; Exception: %s", ex.what());
      success = false;
    }
    clusters[i].header.stamp = cloud_header.stamp;
    clusters[i].header.frame_id = cloud_header.frame_id;
  }
  return success;
}

/*! Performs the detection on each of the clusters, and populates the returned message.
*/
template <class PointCloudType>
void TabletopNode::objectDetection(roslib::Header cloud_header,
				   std::vector<PointCloudType> &clusters,
				   const tf::Transform &table_plane_trans,
				   TabletopDetectionResult &detection_message)
{
  geometry_msgs::Pose table_pose;
  tf::poseTFToMsg(table_plane_trans, table_pose);

  //do the model fitting part
  ROS_INFO("Fitting models to clusters");
  std::vector< std::vector<ModelFitInfo> > raw_fit_results;
  for (size_t i=0; i<clusters.size(); i++) 
  {
    raw_fit_results.push_back( detector_.fitBestModels<PointCloudType>(clusters[i], 1) );
    detection_message.cluster_model_indices[i] = i;
  }

  //merge models that were fit very close to each other
  if (perform_fit_merge_)
  {
    size_t i=0;
    while (i < clusters.size())
    {
      if (raw_fit_results.at(i).empty())
      {
	i++;
	continue;
      }
      size_t j;
      for (j=i+1; j<clusters.size(); j++)
      {
	if (raw_fit_results.at(j).empty()) 
	{
	  if ( fitClusterDistance<PointCloudType>( raw_fit_results.at(i).at(0), 
						   clusters.at(j) ) < fit_merge_threshold_ ) break;
	  else continue;
	}
	if ( fitDistance(raw_fit_results.at(i).at(0), raw_fit_results.at(j).at(0)) < fit_merge_threshold_) break;
      }
      if (j<clusters.size())
      {
	ROS_INFO("Post-fit merging of clusters %u and %u", (unsigned int) i, (unsigned int) j);
	//merge cluster j into i
	clusters[i].points.insert( clusters[i].points.end(), clusters[j].points.begin(), clusters[j].points.end() );
	//delete fits for cluster j so we ignore it from now on
	raw_fit_results.at(j).clear();
	//fits for cluster j now point at fit for cluster i
	detection_message.cluster_model_indices[j] = i;
	//refit cluster i
	raw_fit_results.at(i) = detector_.fitBestModels(clusters[i], 1);
      }
      else
      {
	i++;
      }
    }
  }

  //get just the best fit for each cluster in a separate array
  //also make sure raw clusters point at the right index in fit_models
  std::vector<ModelFitInfo> fit_models;
  for (size_t i=0; i<raw_fit_results.size(); i++)
  {
    if (!raw_fit_results.at(i).empty()) 
    {
      fit_models.push_back( raw_fit_results.at(i).at(0) );
      ROS_INFO("  - fit with score %f for model id %d ", fit_models.back().getScore(), 
	       fit_models.back().getModelId() );
      detection_message.cluster_model_indices[i] = fit_models.size() - 1;
    }
    else
    {
      if (detection_message.cluster_model_indices[i] != (int)i)
      {
	int ind = detection_message.cluster_model_indices[i];
	ROS_ASSERT( ind < (int)i);
	detection_message.cluster_model_indices[i] = detection_message.cluster_model_indices[ind];
	ROS_INFO("  - has been merged with fit for cluster %d", ind);
      }
      else
      {
	ROS_INFO("  - no fit");
	detection_message.cluster_model_indices[i] = -1;
      }
    }

  }

  for (size_t i=0; i<fit_models.size(); i++)
  {
    //get the model pose in the cloud frame by multiplying with table transform 
    tf::Transform model_trans;
    tf::poseMsgToTF(fit_models[i].getPose(), model_trans);
    model_trans = table_plane_trans * model_trans;
    geometry_msgs::Pose model_pose;
    tf::poseTFToMsg(model_trans, model_pose);
    
    //prepare the actual result for good fits, only these are returned
    //also make sure raw clusters (if requested) now point at the right index in the *returned* models
    int model_index;
    if (fit_models[i].getScore() <= quality_threshold_)
    {
      household_objects_database_msgs::DatabaseModelPose pose_message;
      pose_message.model_id = fit_models[i].getModelId();
      //model is published in incoming cloud frame
      pose_message.pose.header = cloud_header;
      pose_message.pose.pose = model_pose;
      //transform is identity since now objects have their own reference frame
      detection_message.models.push_back(pose_message);

      //the raw cluster model index should point at the newly inserted model
      model_index = detection_message.models.size() - 1;
    }
    else
    {
      //the raw cluster model index has no model to point to
      model_index = -1;
    }

    //make sure cluster indices point to the right thing
    //recall that more than one raw cluster can point to this model
    for (size_t j=0; j<detection_message.cluster_model_indices.size(); j++)
    {
      if (detection_message.cluster_model_indices[j] == (int)i)
      {
	detection_message.cluster_model_indices[j] = model_index;
      }
    }

    //fit markers for all fits
    if ( (fit_models[i].getScore() <= quality_threshold_ && publish_good_fit_markers_) || 
	 (fit_models[i].getScore() >  quality_threshold_ && publish_bad_fit_markers_) )
    { 
      household_objects_database_msgs::GetModelMesh get_mesh;
      get_mesh.request.model_id = fit_models[i].getModelId();
      if ( !get_model_mesh_srv_.call(get_mesh) || 
	   get_mesh.response.return_code.code != get_mesh.response.return_code.SUCCESS )
      {
	ROS_ERROR("tabletop_object_detector: failed to call database get mesh service for marker display");
      }
      else
      {
	visualization_msgs::Marker fitMarker =  MarkerGenerator::getFitMarker(get_mesh.response.mesh, 
									      fit_models[i].getScore(), 
									      quality_threshold_);
	fitMarker.header = cloud_header;
	fitMarker.pose = model_pose;
	fitMarker.ns = "tabletop_node";
	fitMarker.id = current_marker_id_++;
	marker_pub_.publish(fitMarker);
      }
    }
  }
}

void TabletopNode::clearOldMarkers(std::string frame_id)
{
  for (int id=current_marker_id_; id < num_markers_published_; id++)
  {
    visualization_msgs::Marker delete_marker;
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.header.frame_id = frame_id;
    delete_marker.id = id;
    delete_marker.action = visualization_msgs::Marker::DELETE;
    delete_marker.ns = "tabletop_node";
    marker_pub_.publish(delete_marker);
  }
  num_markers_published_ = current_marker_id_;
  current_marker_id_ = 0;
}

double TabletopNode::fitDistance(const ModelFitInfo &m1, const ModelFitInfo &m2)
{
  double dx = m1.getPose().position.x - m2.getPose().position.x;
  double dy = m1.getPose().position.y - m2.getPose().position.y;
  double d = dx*dx + dy*dy;
  return sqrt(d);
}

template <class PointCloudType>
double TabletopNode::fitClusterDistance(const ModelFitInfo &m, const PointCloudType &cluster)
{
  double dist = 100.0 * 100.0;
  double mx = m.getPose().position.x;
  double my = m.getPose().position.x;
  for (size_t i=0; i<cluster.points.size(); i++)
  {
    double dx = cluster.points[i].x - mx;
    double dy = cluster.points[i].y - my;
    double d = dx*dx + dy*dy;
    dist = std::min(d, dist);
  }
  return sqrt(dist);
}

/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction)
{
  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  btVector3 position(-a*d, -b*d, -c*d);
  btVector3 z(a, b, c);
  //make sure z points "up"
  ROS_DEBUG("z.dot: %0.3f", z.dot(btVector3(0,0,1)));
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
  if ( z.dot( btVector3(0, 0, up_direction) ) < 0)
  {
    z = -1.0 * z;
    ROS_INFO("flipped z");
  }
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  btVector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = btVector3(0, 1, 0);
  btVector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  btMatrix3x3 rotation;
  rotation[0] = x; 	// x
  rotation[1] = y; 	// y
  rotation[2] = z; 	// z
  rotation = rotation.transpose();
  btQuaternion orientation;
  rotation.getRotation(orientation);
  return tf::Transform(orientation, position);
}

template <typename PointT> 
bool getPlanePoints (const pcl::PointCloud<PointT> &table, 
		     const tf::Transform& table_plane_trans,
		     sensor_msgs::PointCloud &table_points)
{
  // Prepare the output
  table_points.header = table.header;
  table_points.points.resize (table.points.size ());
  for (size_t i = 0; i < table.points.size (); ++i)
  {
    table_points.points[i].x = table.points[i].x;
    table_points.points[i].y = table.points[i].y;
    table_points.points[i].z = table.points[i].z;
  }

  // Transform the data
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, table.header.stamp, 
                                        table.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  std::string error_msg;
  if (!listener.canTransform("table_frame", table_points.header.frame_id, table_points.header.stamp, &error_msg))
  {
    ROS_ERROR("Can not transform point cloud from frame %s to table frame; error %s", 
	      table_points.header.frame_id.c_str(), error_msg.c_str());
    return false;
  }
  try
  {
    listener.transformPointCloud("table_frame", table_points, table_points);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s", 
	      table_points.header.frame_id.c_str(), ex.what());
    return false;
  }
  table_points.header.stamp = table.header.stamp;
  table_points.header.frame_id = "table_frame";
  return true;
}

template <typename PointT> void
getClustersFromPointCloud2 (const pcl::PointCloud<PointT> &cloud_objects, 
			    const tf::Transform& table_plane_trans,
			    const std::vector<pcl::PointIndices> &clusters2, 
			    std::vector<sensor_msgs::PointCloud> &clusters)
{
  // Transform the data
  tf::TransformListener listener;
  tf::StampedTransform table_pose_frame(table_plane_trans, cloud_objects.header.stamp, 
                                        cloud_objects.header.frame_id, "table_frame");
  listener.setTransform(table_pose_frame);
  clusters.resize (clusters2.size ());

  for (size_t i = 0; i < clusters2.size (); ++i)
  {
    clusters[i].header.frame_id = cloud_objects.header.frame_id;
    clusters[i].header.stamp = ros::Time(0);
    clusters[i].points.resize (clusters2[i].indices.size ());
    for (size_t j = 0; j < clusters[i].points.size (); ++j)
    {
      clusters[i].points[j].x = cloud_objects.points[clusters2[i].indices[j]].x;
      clusters[i].points[j].y = cloud_objects.points[clusters2[i].indices[j]].y;
      clusters[i].points[j].z = cloud_objects.points[clusters2[i].indices[j]].z;
    }
    listener.transformPointCloud("table_frame", clusters[i], clusters[i]); 
  }
}

void TabletopNode::processCloud(const sensor_msgs::PointCloud2 &cloud,
				bool return_clusters, bool return_fits,
				TabletopDetectionResult &detection_message)
{
  ROS_INFO("Starting process on new cloud");

  // PCL objects
  KdTreePtr normals_tree_, clusters_tree_;
  pcl::VoxelGrid<Point> grid_, grid_objects_;
  pcl::PassThrough<Point> pass_;
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;
  pcl::ProjectInliers<Point> proj_;
  pcl::ConvexHull2D<Point> hull_;
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::EuclideanClusterExtraction<Point> pcl_cluster_;

  // Filtering parameters
  grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
  grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
  grid_.setFilterFieldName ("z");
  pass_.setFilterFieldName ("z");

  pass_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setFilterLimits (z_filter_min_, z_filter_max_);
  grid_.setDownsampleAllData (false);
  grid_objects_.setDownsampleAllData (false);

  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();

  // Normal estimation parameters
  n3d_.setKSearch (10);  
  n3d_.setSearchMethod (normals_tree_);
  // Table model fitting parameters
  seg_.setDistanceThreshold (0.05); 
  seg_.setMaxIterations (10000);
  seg_.setNormalDistanceWeight (0.1);
  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setProbability (0.99);

  proj_.setModelType (pcl::SACMODEL_PLANE);

  // Consider only objects in a given layer above the table
  prism_.setHeightLimits (table_z_filter_min_, table_z_filter_max_);

  // Clustering parameters
  pcl_cluster_.setClusterTolerance (cluster_distance_);
  pcl_cluster_.setMinClusterSize (min_cluster_size_);
  pcl_cluster_.setSearchMethod (clusters_tree_);

  // Step 1 : Filter, remove NaNs and downsample
  pcl::PointCloud<Point> cloud_t;
  pcl::fromROSMsg (cloud, cloud_t);
  pcl::PointCloud<Point>::ConstPtr cloud_ptr = boost::make_shared<const pcl::PointCloud<Point> > (cloud_t);

  pcl::PointCloud<Point> cloud_filtered;
  pass_.setInputCloud (cloud_ptr);
  pass_.filter (cloud_filtered);
  pcl::PointCloud<Point>::ConstPtr cloud_filtered_ptr = 
    boost::make_shared<const pcl::PointCloud<Point> > (cloud_filtered);
  ROS_INFO("Step 1 done");
  if (cloud_filtered.points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Filtered cloud only has %d points", (int)cloud_filtered.points.size());
    detection_message.result = detection_message.NO_TABLE;
    return;
  }

  pcl::PointCloud<Point> cloud_downsampled;
  grid_.setInputCloud (cloud_filtered_ptr);
  grid_.filter (cloud_downsampled);
  pcl::PointCloud<Point>::ConstPtr cloud_downsampled_ptr = 
    boost::make_shared<const pcl::PointCloud<Point> > (cloud_downsampled);
  if (cloud_downsampled.points.size() < (unsigned int)min_cluster_size_)
  {
    ROS_INFO("Downsampled cloud only has %d points", (int)cloud_downsampled.points.size());
    detection_message.result = detection_message.NO_TABLE;    
    return;
  }

  // Step 2 : Estimate normals
  pcl::PointCloud<pcl::Normal> cloud_normals;
  n3d_.setInputCloud (cloud_downsampled_ptr);
  n3d_.compute (cloud_normals);
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr = 
    boost::make_shared<const pcl::PointCloud<pcl::Normal> > (cloud_normals);
  ROS_INFO("Step 2 done");

  // Step 3 : Perform planar segmentation
  pcl::PointIndices table_inliers; pcl::ModelCoefficients table_coefficients;
  seg_.setInputCloud (cloud_downsampled_ptr);
  seg_.setInputNormals (cloud_normals_ptr);
  seg_.segment (table_inliers, table_coefficients);
  pcl::PointIndices::ConstPtr table_inliers_ptr = boost::make_shared<const pcl::PointIndices> (table_inliers);
  pcl::ModelCoefficients::ConstPtr table_coefficients_ptr = 
    boost::make_shared<const pcl::ModelCoefficients> (table_coefficients);

  if (table_coefficients.values.size () <=3)
  {
    ROS_INFO("Failed to detect table in scan");
    detection_message.result = detection_message.NO_TABLE;    
    return;
  }

  if ( table_inliers.indices.size() < (unsigned int)inlier_threshold_)
  {
    ROS_INFO("Plane detection has %d inliers, below min threshold of %d", (int)table_inliers.indices.size(),
	     inlier_threshold_);
    detection_message.result = detection_message.NO_TABLE;
    return;
  }

  ROS_INFO ("[TableObjectDetector::input_callback] Model found with %d inliers: [%f %f %f %f].", 
	    (int)table_inliers.indices.size (),
	    table_coefficients.values[0], table_coefficients.values[1], 
	    table_coefficients.values[2], table_coefficients.values[3]);
  ROS_INFO("Step 3 done");

  // Step 4 : Project the table inliers on the table
  pcl::PointCloud<Point> table_projected;
  proj_.setInputCloud (cloud_downsampled_ptr);
  proj_.setIndices (table_inliers_ptr);
  proj_.setModelCoefficients (table_coefficients_ptr);
  proj_.filter (table_projected);
  pcl::PointCloud<Point>::ConstPtr table_projected_ptr = 
    boost::make_shared<const pcl::PointCloud<Point> > (table_projected);
  ROS_INFO("Step 4 done");
  
  sensor_msgs::PointCloud table_points;
  tf::Transform table_plane_trans = getPlaneTransform (table_coefficients, up_direction_);
  //takes the points projected on the table and transforms them into the PointCloud message
  //while also transforming them into the table's coordinate system
  if (!getPlanePoints<Point> (table_projected, table_plane_trans, table_points))
  {
    detection_message.result = detection_message.OTHER_ERROR;
    return;
  }
  ROS_INFO("Table computed");

  detection_message.table = getTable<sensor_msgs::PointCloud>(cloud.header, table_plane_trans, table_points);
  detection_message.result = detection_message.SUCCESS;

  // ---[ Estimate the convex hull
  pcl::PointCloud<Point> table_hull;
  hull_.setInputCloud (table_projected_ptr);
  hull_.reconstruct (table_hull);
  pcl::PointCloud<Point>::ConstPtr table_hull_ptr = boost::make_shared<const pcl::PointCloud<Point> > (table_hull);

  // ---[ Get the objects on top of the table
  pcl::PointIndices cloud_object_indices;
  //prism_.setInputCloud (cloud_all_minus_table_ptr);
  prism_.setInputCloud (cloud_filtered_ptr);
  prism_.setInputPlanarHull (table_hull_ptr);
  prism_.segment (cloud_object_indices);

  pcl::PointCloud<Point> cloud_objects;
  pcl::ExtractIndices<Point> extract_object_indices;
  extract_object_indices.setInputCloud (cloud_filtered_ptr);
  extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
  extract_object_indices.filter (cloud_objects);
  pcl::PointCloud<Point>::ConstPtr cloud_objects_ptr = boost::make_shared<const pcl::PointCloud<Point> > (cloud_objects);
  ROS_INFO (" Number of object point candidates: %d.", (int)cloud_objects.points.size ());

  if (cloud_objects.points.empty ()) 
  {
    ROS_INFO("No objects on table");
    return;
  }

  // ---[ Downsample the points
  pcl::PointCloud<Point> cloud_objects_downsampled;
  grid_objects_.setInputCloud (cloud_objects_ptr);
  grid_objects_.filter (cloud_objects_downsampled);
  pcl::PointCloud<Point>::ConstPtr cloud_objects_downsampled_ptr 
    = boost::make_shared <const pcl::PointCloud<Point> > (cloud_objects_downsampled);

  // ---[ Split the objects into Euclidean clusters
  std::vector<pcl::PointIndices> clusters2;
  //pcl_cluster_.setInputCloud (cloud_objects_ptr);
  pcl_cluster_.setInputCloud (cloud_objects_downsampled_ptr);
  pcl_cluster_.extract (clusters2);
  ROS_INFO ("Number of clusters found matching the given constraints: %d.", (int)clusters2.size ());

  //converts clusters back into the PointCloud message while also transforming them into
  //the table reference frame
  std::cerr << "cloud_objects frame: " << cloud_objects_ptr->header.frame_id << std::endl;
  std::vector<sensor_msgs::PointCloud> clusters;
  //getClustersFromPointCloud2<Point> (cloud_objects, table_plane_trans, clusters2, clusters);
  getClustersFromPointCloud2<Point> (cloud_objects_downsampled, table_plane_trans, clusters2, clusters);
  ROS_INFO("Clusters converted");

  detection_message.cluster_model_indices = std::vector<int>(clusters.size(), -1);

  if (return_clusters)
  {
    detection_message.clusters = clusters;
    //transforms the clusters back in the frame of the original point cloud
    //this is wasteful, as we do start with the point cloud in its original frame
    //but it was just easier this way
    if (!transformClustersToCloudFrame(detection_message.clusters, cloud.header, table_plane_trans))
    {
      detection_message.result = detection_message.OTHER_ERROR;
      return;
    }
  }

  if (return_fits)
  {
    if (!use_database_)
    {
      ROS_ERROR("Recognition results requested, but database use is disabled for the detector");
    }
    else
    {
      //also makes sure that cluster_model_indices points at the right thing
      objectDetection<sensor_msgs::PointCloud>(cloud.header, clusters, table_plane_trans, detection_message);
    }
  }

  //publish cluster markers now as we might have merged stuff
  if (publish_cluster_markers_)
  {
    publishClusterMarkers<sensor_msgs::PointCloud>(clusters, cloud.header, table_plane_trans);
  }

  clearOldMarkers(cloud.header.frame_id);
}


} //namespace tabletop_object_detector

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "tabletop_node");
  ros::NodeHandle nh;

  tabletop_object_detector::TabletopNode node(nh);
  ros::spin();
  return 0;
}
