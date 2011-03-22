/**
 * @file   sr_grasp_planner.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Mar 22 10:39:10 2011
 *
 * @brief  Plans grasps for an unknow object.
 *
 *
 */

#ifndef _SR_GRASP_PLANNER_H_
#define _SR_GRASP_PLANNER_H_

#include <ros/ros.h>
#include <object_manipulation_msgs/Grasp.h>
#include <object_manipulation_msgs/GraspableObject.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

namespace shadowrobot
{
  class SrGraspPlanner
  {
  public:
    SrGraspPlanner();
    ~SrGraspPlanner();

    std::vector<object_manipulation_msgs::Grasp> compute_list_of_grasps(object_manipulation_msgs::GraspableObject target, geometry_msgs::Pose current_pose);

  protected:
    sensor_msgs::JointState pregrasp, grasp;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
