/**
 * @file   move_arm_simple_action.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jan 26 10:32:10 2011
 *
 * @brief  
 *
 */

#ifndef SR_MOVE_ARM_SIMPLE_ACTION_H
#define SR_MOVE_ARM_SIMPLE_ACTION_H

#include <ros/ros.h>
#include <boost/smart_ptr.hpp>
#include <sr_hand/sendupdate.h>

#include <actionlib/server/simple_action_server.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/MoveArmResult.h>
#include <move_arm_msgs/MoveArmStatistics.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <object_manipulation_msgs/GraspStatus.h>

using namespace ros;

namespace shadowrobot
{
  class SrMoveArmSimpleAction
  {
  public:
    SrMoveArmSimpleAction();
    ~SrMoveArmSimpleAction();

  protected:
    NodeHandle nh, nh_tilde;
    Publisher sr_arm_target_pub;
    Publisher sr_hand_target_pub;

    void execute(const move_arm_msgs::MoveArmGoalConstPtr& goal);
    void execute_trajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr& goal);

    boost::shared_ptr<actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction> > action_server;
    boost::shared_ptr<actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> > action_server_joint_trajectory;
    ros::ServiceServer grasp_status_server;
    bool check_grasp_status(object_manipulation_msgs::GraspStatus::Request &req, object_manipulation_msgs::GraspStatus::Response &res);

    sr_hand::sendupdate sendupdate_msg;
    std::vector<sr_hand::joint> joint_vector;
    
    move_arm_msgs::MoveArmResult move_arm_action_result;
    move_arm_msgs::MoveArmFeedback move_arm_action_feedback;
  };//end class
}//end workspace

#endif SR_MOVE_ARM_SIMPLE_ACTION_H

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
