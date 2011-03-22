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

namespace shadowrobot
{
  SrGraspPlanner::SrGraspPlanner()
  {
    ROS_ERROR("TODO: Initialize the Grasp Planner");

    pregrasp.name.push_back("FFJ3");
    grasp.name.push_back("FFJ3");
  }

  SrGraspPlanner::~SrGraspPlanner()
  {}

  std::vector<object_manipulation_msgs::Grasp> SrGraspPlanner::compute_list_of_grasps(object_manipulation_msgs::GraspableObject target, geometry_msgs::Pose current_pose)
  {
    std::vector<object_manipulation_msgs::Grasp> possible_grasps;

    ROS_ERROR("TODO: Compute possible grasps");

    pregrasp.position.push_back(0.0);
    grasp.position.push_back(90.0);
    object_manipulation_msgs::Grasp tmp_grasp;
    tmp_grasp.pre_grasp_posture = pregrasp;
    tmp_grasp.grasp_posture = grasp;
    tmp_grasp.grasp_pose = current_pose;
    possible_grasps.push_back(tmp_grasp);

    return possible_grasps;
  }

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
