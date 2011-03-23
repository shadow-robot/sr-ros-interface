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

namespace shadowrobot
{
  const double SrGraspPlanner::default_approach_distance = 0.1;
  const unsigned short SrGraspPlanner::default_number_of_computed_grasps = 1;

  SrGraspPlanner::SrGraspPlanner()
  {
    pregrasp.name.push_back("FFJ3");
    grasp.name.push_back("FFJ3");
  }

  SrGraspPlanner::~SrGraspPlanner()
  {}

  std::vector<object_manipulation_msgs::Grasp> SrGraspPlanner::compute_list_of_grasps(object_manipulation_msgs::ClusterBoundingBox bounding_box, geometry_msgs::Pose current_pose)
  {
    //get the main axis
    Eigen::Vector3d main_axis = get_main_axis(bounding_box);

    std::vector<object_manipulation_msgs::Grasp> possible_grasps;

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

      //this is a "no rotation" starting point
      tf::Quaternion current_rotation;
      current_rotation[0] = 0.0;
      current_rotation[1] = 0.0;
      current_rotation[2] = 0.0;
      current_rotation[3] = -1.0;

      //the default grasp comes from above the main axis
      if (main_axis[2] == 1)
      {
        ROS_INFO("Computing grasps for a vertical object");
        tmp_grasp.grasp_pose.position.x += default_approach_distance;

        tf::Quaternion rot1(tf::Vector3(1,0,0), M_PI);
        tf::Quaternion rot2(tf::Vector3(0,1,0), M_PI / 2.0);

        current_rotation *= rot1;
        current_rotation.normalize();
        current_rotation *= rot2;
        current_rotation.normalize();
      }
      else
      {
        ROS_INFO("Computing grasps for an horizontal object");
        tmp_grasp.grasp_pose.position.z += default_approach_distance;
      }
      tmp_grasp.grasp_pose.orientation.x = current_rotation[0];
      tmp_grasp.grasp_pose.orientation.y = current_rotation[1];
      tmp_grasp.grasp_pose.orientation.z = current_rotation[2];
      tmp_grasp.grasp_pose.orientation.w = current_rotation[3];

      possible_grasps.push_back(tmp_grasp);
    }

    return possible_grasps;
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
