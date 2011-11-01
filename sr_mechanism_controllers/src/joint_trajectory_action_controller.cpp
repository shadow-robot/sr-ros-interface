/**
 * @file   joint_trajectory_action_controller.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Mar  4 13:08:22 2011
 * 
 * @brief  Implement an actionlib server to execute a 
 * pr2_controllers_msgs::JointTrajectoryAction. Follows the 
 * given trajectory with the arm.
 * 
 * 
 */

#include "sr_mechanism_controllers/joint_trajectory_action_controller.hpp"

#include <sr_robot_msgs/sendupdate.h>

namespace shadowrobot
{
  JointTrajectoryActionController::JointTrajectoryActionController() :
    nh_tilde("~")
  {
    action_server = boost::shared_ptr<JTAS> (new JTAS("/r_arm_controller/joint_trajectory_action", 
                                                      boost::bind(&JointTrajectoryActionController::execute_trajectory, this, _1), 
                                                      false ));

    sr_arm_target_pub = nh.advertise<sr_robot_msgs::sendupdate>("/sr_arm/sendupdate", 2);
    sr_hand_target_pub = nh.advertise<sr_robot_msgs::sendupdate>("/srh/sendupdate", 2);

    action_server->start();
  }

  JointTrajectoryActionController::~JointTrajectoryActionController()
  {
    
  }

  void JointTrajectoryActionController::execute_trajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr& goal)
  {
    bool success = true;

    sr_robot_msgs::sendupdate sendupdate_msg_traj;
    std::vector<sr_robot_msgs::joint> joint_vector_traj;
    
    //initializes the joint names
    std::vector<std::string> joint_names = goal->trajectory.joint_names;
    joint_vector_traj.clear();
    for(unsigned int i = 0; i < joint_names.size(); ++i)
    {
      sr_robot_msgs::joint joint;
      joint.joint_name = joint_names[i];
      joint_vector_traj.push_back(joint);
    }
    sendupdate_msg_traj.sendupdate_length = joint_vector_traj.size();

    ROS_DEBUG("Trajectory received: %d joints / %d msg length", (int)goal->trajectory.joint_names.size(), sendupdate_msg_traj.sendupdate_length);

    ros::Rate tmp_rate(1.0);
    
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = goal->trajectory.points;
    trajectory_msgs::JointTrajectoryPoint trajectory_step;
    
    //loop through the steps
    ros::Duration sleeping_time(0.0), last_time(0.0);
    for(unsigned int index_step = 0; index_step < trajectory_points.size(); ++index_step)
    {
      trajectory_step = trajectory_points[index_step];

      //check if preempted
      if (action_server->isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Joint Trajectory Action Preempted");
        // set the action state to preempted
        action_server->setPreempted();
        success = false;
        break;
      }

      //update the targets
      for(unsigned int index_pos = 0; index_pos < (unsigned int)sendupdate_msg_traj.sendupdate_length; ++index_pos)
      {
        joint_vector_traj[index_pos].joint_target = trajectory_step.positions[index_pos] * 57.3;

        ROS_DEBUG("traj[%s]: %f", joint_vector_traj[index_pos].joint_name.c_str(), joint_vector_traj[index_pos].joint_target);

      }
      sendupdate_msg_traj.sendupdate_list = joint_vector_traj;
      
      sr_arm_target_pub.publish(sendupdate_msg_traj);
      sr_hand_target_pub.publish(sendupdate_msg_traj);

      sleeping_time.sleep();
      sleeping_time = trajectory_step.time_from_start - last_time + ros::Duration(0.05);
      last_time = trajectory_step.time_from_start;
    }
    
    pr2_controllers_msgs::JointTrajectoryResult joint_trajectory_result;
    if(success)
      action_server->setSucceeded(joint_trajectory_result);
    else
      action_server->setAborted(joint_trajectory_result);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_joint_trajectory_action_controller");

  ros::AsyncSpinner spinner(1); //Use 1 thread
  spinner.start();
  shadowrobot::JointTrajectoryActionController jac;
  ros::spin();
  //ros::waitForShutdown();

  return 0;
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
