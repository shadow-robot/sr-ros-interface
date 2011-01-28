/**
 * @file   move_arm_simple_action.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jan 26 10:32:10 2011
 *
 * @brief  
 *
 */

#include "sr_move_arm/move_arm_simple_action.hpp"

#include <sr_hand/joints_data.h>
#include <sr_hand/joint.h>


namespace shadowrobot
{

  SrMoveArmSimpleAction::SrMoveArmSimpleAction() :
    nh_tilde("~")
  {
    action_server = boost::shared_ptr<actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction> >(new actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction>("/right_arm/move_arm", boost::bind(&SrMoveArmSimpleAction::execute, this, _1)));

    sr_arm_target_pub = nh.advertise<sr_hand::sendupdate>("/sr_arm/sendupdate", 2);
    sr_hand_target_pub = nh.advertise<sr_hand::sendupdate>("/sr_hand/sendupdate", 2);

    action_server->start();
  }

  SrMoveArmSimpleAction::~SrMoveArmSimpleAction()
  {}

  void SrMoveArmSimpleAction::execute(const move_arm_msgs::MoveArmGoalConstPtr& goal)
  {
    ros::Rate ts(1.0);
    move_arm_action_feedback.time_to_completion = ts.cycleTime();
    move_arm_action_feedback.state = "stopped";
    action_server->publishFeedback(move_arm_action_feedback);
    
    bool arm_in_correct_position = true;

    if(action_server->isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Move arm action preempted.");
      //set action state to preempted
      action_server->setPreempted();
      arm_in_correct_position = false;
    }
      
    //TODO: compute an approximate remaining time
    move_arm_action_feedback.time_to_completion = ros::Rate(1.0).cycleTime();
    move_arm_action_feedback.state = "moving";
    action_server->publishFeedback(move_arm_action_feedback);
    
    joint_vector.clear();
    for( unsigned int i=0; i < goal->motion_plan_request.goal_constraints.joint_constraints.size(); ++i )
    {
      sr_hand::joint joint;
      joint.joint_name = goal->motion_plan_request.goal_constraints.joint_constraints[i].joint_name;
      joint.joint_target = goal->motion_plan_request.goal_constraints.joint_constraints[i].position * 57.29583;

      joint_vector.push_back(joint);
    }

    sendupdate_msg.sendupdate_length = joint_vector.size();
    sendupdate_msg.sendupdate_list = joint_vector;

    sr_arm_target_pub.publish(sendupdate_msg);
    sr_hand_target_pub.publish(sendupdate_msg);

    ts.sleep();
    arm_in_correct_position = true;

    if(arm_in_correct_position)
    {
      move_arm_action_feedback.time_to_completion = ros::Rate(0.0).cycleTime();
      move_arm_action_feedback.state = "stopped";
      action_server->publishFeedback(move_arm_action_feedback);
      move_arm_action_result.error_code.val = 0;
      action_server->setSucceeded(move_arm_action_result);
    }

  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_move_arm");

  //ros::AsyncSpinner spinner(1); //Use 1 thread
  //spinner.start();
  shadowrobot::SrMoveArmSimpleAction move_arm;
  ros::spin();
  //ros::waitForShutdown();

  return 0;
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
