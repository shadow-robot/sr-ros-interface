/**
 * @file   joint_trajectory_action_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Mar  4 12:57:46 2011
 *
 * @brief  Implement an actionlib server to execute a
 * control_msgs::JointTrajectoryAction. Follows the
 * given trajectory with the arm.
 *
 *
 */

#ifndef _SR_JOINT_TRAJECTORY_ACTION_CONTROLLER_H_
#define _SR_JOINT_TRAJECTORY_ACTION_CONTROLLER_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace shadowrobot
{
  class JointTrajectoryActionController
  {
    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
    typedef std::vector<trajectory_msgs::JointTrajectoryPoint> JointTrajectoryPointVec;
    typedef std::map<std::string, ros::Publisher> JointPubMap;
  public:
    JointTrajectoryActionController();

  private:
    ros::NodeHandle nh;
    JointPubMap    joint_pub;
    boost::shared_ptr<JTAS> action_server;

    void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
