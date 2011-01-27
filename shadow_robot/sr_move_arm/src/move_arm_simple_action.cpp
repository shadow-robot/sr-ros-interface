/**
 * @file   move_arm_simple_action.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jan 26 10:32:10 2011
 *
 * @brief  
 *
 */

#include "sr_move_arm/move_arm_simple_action.hpp"

namespace shadowrobot
{

  SrMoveArmSimpleAction::SrMoveArmSimpleAction() :
    nh_tilde("~")
  {
    action_server = boost::shared_ptr<actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction> >(new actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction>("/right_arm/move_arm", boost::bind(&SrMoveArmSimpleAction::execute, this, _1)));

    action_server->start();
  }

  SrMoveArmSimpleAction::~SrMoveArmSimpleAction()
  {}

  void SrMoveArmSimpleAction::execute(const move_arm_msgs::MoveArmGoalConstPtr& Goal)
  {
    ros::Rate ts(0.2);
    move_arm_action_feedback.time_to_completion = ts.cycleTime();
    move_arm_action_feedback.state = "stopped";
    action_server->publishFeedback(move_arm_action_feedback);
    
    bool arm_in_correct_position = true;

    ts = ros::Rate(1.0);

    //TODO: move the arm 
    for(unsigned int i=0;i<5; ++i)
    {
      if(action_server->isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Move arm action preempted.");
        //set action state to preempted
        action_server->setPreempted();
        arm_in_correct_position = false;
      }
      
      move_arm_action_feedback.time_to_completion = ros::Rate(5.0 - (double)i).cycleTime();
      move_arm_action_feedback.state = "stopped";
      action_server->publishFeedback(move_arm_action_feedback);
    
      ts.sleep();
    }

    if(arm_in_correct_position)
    {
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
