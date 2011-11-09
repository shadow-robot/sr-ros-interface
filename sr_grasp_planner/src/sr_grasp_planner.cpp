/**
 * @file   sr_grasp_planner.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Mar 22 10:37:43 2011
 *
 * @brief  Plans grasp for an object that was not recognized.
 *
 *
 */

#include "sr_grasp_planner/sr_grasp_planner.hpp"
#include <math.h>
#include <tf/tf.h>
#include <sstream>

namespace shadowrobot
{
  const double SrGraspPlanner::default_approach_distance = 0.2;
  const unsigned short SrGraspPlanner::default_number_of_computed_grasps = 20;

  SrGraspPlanner::SrGraspPlanner()
  {
    pregrasp.name.push_back("FFJ3");
    grasp.name.push_back("FFJ3");
  }

  SrGraspPlanner::~SrGraspPlanner()
  {}

  std::vector<object_manipulation_msgs::Grasp> SrGraspPlanner::compute_list_of_grasps(object_manipulation_msgs::ClusterBoundingBox bounding_box)
  {
    //get the main axis
    Eigen::Vector3d main_axis = get_main_axis(bounding_box);

    std::vector<object_manipulation_msgs::Grasp> possible_grasps;

    tf::Quaternion object_rotation(bounding_box.pose_stamped.pose.orientation.x,
                                   bounding_box.pose_stamped.pose.orientation.y,
                                   bounding_box.pose_stamped.pose.orientation.z,
                                   bounding_box.pose_stamped.pose.orientation.w);

    //compute the grasps: they're placed on a cylinder surounding the
    //biggest axis
    for (unsigned short i = 0; i < default_number_of_computed_grasps; ++i)
    {
      object_manipulation_msgs::Grasp tmp_grasp;

      pregrasp.position.push_back(0.0);
      grasp.position.push_back(90.0);
      tmp_grasp.pre_grasp_posture = pregrasp;
      tmp_grasp.grasp_posture = grasp;

      tmp_grasp.grasp_pose = bounding_box.pose_stamped.pose;

      //the default grasp comes from above the main axis
      if (main_axis[2] == 1)
      {
        compute_pose(i, true, bounding_box, object_rotation);
      }
      else
      {
        compute_pose(i, true, bounding_box, object_rotation);
        //tmp_grasp.grasp_pose.position.z += default_approach_distance;
      }

      //     tmp_grasp.grasp_pose.orientation = pickup_pose_in_base_link_frame.getOrientation();
      //tmp_grasp.grasp_pose = pickup_pose_in_base_link_frame.pose;
/*
  tmp_grasp.grasp_pose.orientation.x = current_rotation[0];
  tmp_grasp.grasp_pose.orientation.y = current_rotation[1];
  tmp_grasp.grasp_pose.orientation.z = current_rotation[2];
  tmp_grasp.grasp_pose.orientation.w = current_rotation[3];
*/
      possible_grasps.push_back(tmp_grasp);
    }

    return possible_grasps;
  }

  void SrGraspPlanner::compute_pose(unsigned int index_pose, bool is_vertical,
                                    object_manipulation_msgs::ClusterBoundingBox bounding_box,
                                    tf::Quaternion object_rotation)
  {
    geometry_msgs::PoseStamped pickup_pose_in_object_frame, pickup_pose_in_base_link_frame;

    if( index_pose == 0)
    {
      if( is_vertical )
        ROS_INFO("Computing grasps for a vertical object");
      else
        ROS_INFO("Computing grasps for an horizontal object");
    }

    tf::Quaternion current_rotation;
    if( index_pose == 0 ) //no rotation
    {
      current_rotation[0] = 0.0;
      current_rotation[1] = 0.0;
      current_rotation[2] = 0.0;
      current_rotation[3] = 1.0;
    }
    else
      current_rotation.setRPY(0, 0, 2.0 * M_PI * double(index_pose) / double(default_number_of_computed_grasps) );

    std::string pose_to_pickup_object_name = "/pose_to_pickup_object_";
    std::stringstream ss;
    ss << pose_to_pickup_object_name << index_pose;
    pose_to_pickup_object_name = ss.str();

    //broadcast the object transform
    tf::Transform object_transform;
    object_transform.setOrigin( tf::Vector3(bounding_box.pose_stamped.pose.position.x,
                                            bounding_box.pose_stamped.pose.position.y,
                                            bounding_box.pose_stamped.pose.position.z) );

    current_rotation *= object_rotation;
    current_rotation.normalize();

    ROS_DEBUG_STREAM("ROT: "
                     << current_rotation[0] << " "
                     << current_rotation[1] << " "
                     << current_rotation[2] << " "
                     << current_rotation[3]  );

    object_transform.setRotation( current_rotation );
    tf::Transform pickup_tf_object_frame;
    pickup_tf_object_frame.setOrigin( tf::Vector3(0.1,0,0) );
    pickup_tf_object_frame.setRotation( tf::Quaternion(0,0,0,1) );

    std::string object_tf_name = "/object_to_pickup_";
    std::stringstream ss2;
    ss2 << object_tf_name << index_pose;
    object_tf_name = ss2.str();

    //this is the transform between base_link and the object (based on the bounding box)
    tf_broadcaster.sendTransform( tf::StampedTransform(object_transform,
                                                       ros::Time::now(),
                                                       "/base_link", object_tf_name) );
    tf_listener.waitForTransform("/base_link", object_tf_name,
                                 ros::Time(), ros::Duration(1.0) );
    tf_broadcaster.sendTransform( tf::StampedTransform(pickup_tf_object_frame,
                                                       ros::Time::now(),
                                                       object_tf_name,
                                                       pose_to_pickup_object_name));

    tf::Vector3 v1 = pickup_tf_object_frame.getOrigin();
    tf::Quaternion q1 = pickup_tf_object_frame.getRotation();
    pickup_pose_in_object_frame.header.frame_id = pose_to_pickup_object_name;
    pickup_pose_in_object_frame.pose.position.x = v1.getX();
    pickup_pose_in_object_frame.pose.position.y = v1.getY();
    pickup_pose_in_object_frame.pose.position.z = v1.getZ();

    pickup_pose_in_object_frame.pose.orientation.x = q1.getX();
    pickup_pose_in_object_frame.pose.orientation.y = q1.getY();
    pickup_pose_in_object_frame.pose.orientation.z = q1.getZ();
    pickup_pose_in_object_frame.pose.orientation.w = q1.getW();

    //ros::Rate tmp_rate(10);
    //tmp_rate.sleep();

    //tf_listener.transformPose("base_link", pickup_pose_in_object_frame, pickup_pose_in_base_link_frame );
    //tf_listener.transformPose(object_tf_name, pickup_pose_in_object_frame, pickup_pose_in_base_link_frame );


    ROS_DEBUG_STREAM("ROT: "
                     << pickup_pose_in_base_link_frame.pose.orientation.x << " "
                     << pickup_pose_in_base_link_frame.pose.orientation.y << " "
                     << pickup_pose_in_base_link_frame.pose.orientation.z << " "
                     << pickup_pose_in_base_link_frame.pose.orientation.w << " " );


    //ros::Rate tmp_rate(10);
    //tmp_rate.sleep();
    return;

    tf::StampedTransform pickup_tf_in_base_link_frame;
    ROS_INFO("Waiting for transform from base_link to /pose_to_pickup_object");
    tf_listener.waitForTransform("/base_link", pose_to_pickup_object_name,
                                 ros::Time(), ros::Duration(1.0) );
    tf_listener.lookupTransform("/base_link", pose_to_pickup_object_name,
                                ros::Time(), pickup_tf_in_base_link_frame);

    tf::Vector3 v = pickup_tf_in_base_link_frame.getOrigin();
    tf::Quaternion q = pickup_tf_in_base_link_frame.getRotation();
    pickup_pose_in_base_link_frame.pose.position.x = v.getX();
    pickup_pose_in_base_link_frame.pose.position.y = v.getY();
    pickup_pose_in_base_link_frame.pose.position.z = v.getZ();

    pickup_pose_in_base_link_frame.pose.orientation.x = q.getX();
    pickup_pose_in_base_link_frame.pose.orientation.y = q.getY();
    pickup_pose_in_base_link_frame.pose.orientation.z = q.getZ();
    pickup_pose_in_base_link_frame.pose.orientation.w = q.getW();
    /*
      tf::Quaternion rot1(tf::Vector3(1,0,0), M_PI);
      tf::Quaternion rot2(tf::Vector3(0,1,0), M_PI / 2.0);

      current_rotation *= rot1;
      current_rotation.normalize();
      current_rotation *= rot2;
      current_rotation.normalize();

      tmp_grasp.grasp_pose.position.z += default_approach_distance;
    */

  }

  Eigen::Vector3d SrGraspPlanner::get_main_axis(object_manipulation_msgs::ClusterBoundingBox bbox)
  {
    Eigen::Vector3d main_axis;
    double max = 0;
    if( fabs(bbox.dimensions.x) > max )
    {
      max = bbox.dimensions.x;
      main_axis[0] = 1.0;
    }
    if( fabs(bbox.dimensions.y) > max )
    {
      max = bbox.dimensions.y;
      main_axis[1] = 1.0;
    }
    if( fabs(bbox.dimensions.z) > max )
    {
      max = bbox.dimensions.z;
      main_axis[2] = 1.0;
    }

    return main_axis;
  }

}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
