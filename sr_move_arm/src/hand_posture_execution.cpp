/**
 * @file   hand_posture_execution.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jan 26 10:32:10 2011
 *
 * @brief  Sends target for the Dextrous hand to go into a given grasp or pregrasp position.
 *
 */

#include "sr_move_arm/hand_posture_execution.hpp"

namespace shadowrobot
{

  SrHandPostureExecutionSimpleAction::SrHandPostureExecutionSimpleAction() :
    nh_tilde("~"), hand_occupied(false)
  {
    is_hand_occupied_client = nh.serviceClient<sr_robot_msgs::is_hand_occupied>("is_hand_occupied");

    action_server = boost::shared_ptr<actionlib::SimpleActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction> >(new actionlib::SimpleActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction>("/right_arm/hand_posture_execution", boost::bind(&SrHandPostureExecutionSimpleAction::execute, this, _1), false));

    get_status_server = nh.advertiseService("/right_arm/grasp_status", &SrHandPostureExecutionSimpleAction::getStatusCallback, this);

    sr_hand_target_pub = nh.advertise<sr_robot_msgs::sendupdate>("/srh/sendupdate", 2);

    action_server->start();
  }

  SrHandPostureExecutionSimpleAction::~SrHandPostureExecutionSimpleAction()
  {}

  bool SrHandPostureExecutionSimpleAction::getStatusCallback(object_manipulation_msgs::GraspStatus::Request &request,
                                                             object_manipulation_msgs::GraspStatus::Response &response)
  {
    response.is_hand_occupied = false;

    sr_robot_msgs::is_hand_occupied srv;
    if( is_hand_occupied_client.call(srv) )
    {
      response.is_hand_occupied = srv.response.hand_occupied;
      if( srv.response.hand_occupied)
        ROS_INFO("The hand is occupied");
      else
        ROS_INFO("The hand is empty");
    }
    else
    {
      ROS_ERROR("Called to is_hand_occupied service failed.");
      return false;
    }
    return true;
  }

  void SrHandPostureExecutionSimpleAction::execute(const object_manipulation_msgs::GraspHandPostureExecutionGoalConstPtr& goal)
  {
    if(action_server->isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Change Hand Pose action preempted.");
      //set action state to preempted
      action_server->setPreempted();
    }


    std::vector<std::string> joint_names = goal->grasp.pre_grasp_posture.name;

    joint_vector.clear();
    for(unsigned int i = 0; i < joint_names.size(); ++i)
    {
      sr_robot_msgs::joint joint;
      joint.joint_name = joint_names[i];
      joint_vector.push_back(joint);
    }
    sendupdate_msg.sendupdate_length = joint_vector.size();

    object_manipulation_msgs::GraspHandPostureExecutionResult result;

    switch (goal->goal)
    {
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP:
      ROS_DEBUG("GRASP!");

      if (goal->grasp.grasp_posture.position.empty())
      {
	ROS_ERROR("Shadow Robot grasp execution: position vector empty in requested grasp");
	action_server->setAborted();
	return;
      }
      for(unsigned int i = 0; i < goal->grasp.grasp_posture.position.size(); ++i)
      {
        joint_vector[i].joint_target = goal->grasp.grasp_posture.position[i]*180.0/M_PI;
        ROS_DEBUG("[%s]: %f", joint_names[i].c_str(), joint_vector[i].joint_target);
      }
      sendupdate_msg.sendupdate_list = joint_vector;

      sr_hand_target_pub.publish(sendupdate_msg);
      ROS_DEBUG("Hand in grasp position");

      result.result.value = object_manipulation_msgs::ManipulationResult::SUCCESS;
      action_server->setSucceeded( result );
      hand_occupied = true;

      break;

    case object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP:
      ROS_DEBUG("PREGRASP!");

      if (goal->grasp.pre_grasp_posture.position.empty())
      {
	ROS_ERROR("Shadow Robot grasp execution: position vector empty in requested pre_grasp");
	action_server->setAborted();
	return;
      }
      //move to pregrasp
      for(unsigned int i = 0; i < goal->grasp.pre_grasp_posture.position.size(); ++i)
      {
        joint_vector[i].joint_target = goal->grasp.pre_grasp_posture.position[i]*180.0/M_PI;
        ROS_DEBUG("[%s]: %f", joint_names[i].c_str(), joint_vector[i].joint_target);
      }
      sendupdate_msg.sendupdate_list = joint_vector;

      sr_hand_target_pub.publish(sendupdate_msg);
      ROS_DEBUG("Hand in pregrasp position");

      result.result.value = object_manipulation_msgs::ManipulationResult::SUCCESS;
      action_server->setSucceeded( result );

      hand_occupied = false;

      break;
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE:
      ROS_DEBUG("RELEASE!");

      //open the hand completely
      for(unsigned int i = 0; i < goal->grasp.pre_grasp_posture.position.size(); ++i)
      {
        joint_vector[i].joint_target = 0.0;
        ROS_DEBUG("[%s]: %f", joint_names[i].c_str(), joint_vector[i].joint_target);
      }
      sendupdate_msg.sendupdate_list = joint_vector;

      sr_hand_target_pub.publish(sendupdate_msg);
      ROS_DEBUG("Hand opened");

      result.result.value = object_manipulation_msgs::ManipulationResult::SUCCESS;
      action_server->setSucceeded( result );

      hand_occupied = false;

      break;
    default:
      ROS_ERROR("Shadow Robot grasp execution: unknown goal code (%d)", goal->goal);
      action_server->setAborted();
      return;
    }

    //TODO: check the actual state of the hand and compare to sent targets?
    //object_manipulation_msgs::GraspHandPostureExecutionResult result;
    //result.result.value = object_manipulation_msgs::ManipulationResult::SUCCESS;
    //action_server->setSucceeded( result );
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
