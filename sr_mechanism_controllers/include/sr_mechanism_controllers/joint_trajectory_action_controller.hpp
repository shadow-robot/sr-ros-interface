/**
 * @file   joint_trajectory_action_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Mar  4 12:57:46 2011
 * 
 * @brief  Implement an actionlib server to execute a 
 * pr2_controllers_msgs::JointTrajectoryAction. Follows the 
 * given trajectory with the arm.
 * 
 * 
 */

#ifndef _SR_JOINT_TRAJECTORY_ACTION_CONTROLLER_H_
#define _SR_JOINT_TRAJECTORY_ACTION_CONTROLLER_H_
#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>



namespace shadowrobot 
{
  class JointTrajectoryActionController
  {
    typedef actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
  public:
    JointTrajectoryActionController();
    ~JointTrajectoryActionController();

  private:
    ros::NodeHandle nh, nh_tilde;
    ros::Publisher sr_arm_target_pub;
    ros::Publisher sr_hand_target_pub;
 
    boost::shared_ptr<JTAS> action_server;

    void execute_trajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr& goal);
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
