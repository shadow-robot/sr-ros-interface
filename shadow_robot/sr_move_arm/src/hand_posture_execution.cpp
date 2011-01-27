/**
 * @file   hand_posture_execution.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jan 26 10:32:10 2011
 *
 * @brief  
 *
 */

#include "sr_move_arm/hand_posture_execution.hpp"

namespace shadowrobot
{

  SrHandPostureExecutionSimpleAction::SrHandPostureExecutionSimpleAction() :
    nh_tilde("~")
  {
    action_server = boost::shared_ptr<actionlib::SimpleActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction> >(new actionlib::SimpleActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction>("/right_arm/hand_posture_execution", boost::bind(&SrHandPostureExecutionSimpleAction::execute, this, _1)));

    action_server->start();
  }

  SrHandPostureExecutionSimpleAction::~SrHandPostureExecutionSimpleAction()
  {}

  void SrHandPostureExecutionSimpleAction::execute(const object_manipulation_msgs::GraspHandPostureExecutionGoalConstPtr& Goal)
  {    
    bool hand_in_correct_pose = true;

    ros::Rate ts = ros::Rate(1.0);
    for(unsigned int i=0;i<2; ++i)
    {
      if(action_server->isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Change Hand Pose action preempted.");
        //set action state to preempted
        action_server->setPreempted();
      }
      ts.sleep();
    }

    //TODO: Move the hand to the correct position

    if(hand_in_correct_pose)
    {
      action_server->setSucceeded();
    }
    else
    {
      action_server->setAborted();
    }

  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_posture_execution");

  //ros::AsyncSpinner spinner(1); //Use 1 thread
  //spinner.start();
  shadowrobot::SrHandPostureExecutionSimpleAction move_arm;
  ros::spin();
  //ros::waitForShutdown();

  return 0;
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
