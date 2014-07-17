/**
 * @file   joint_trajectory_action_controller.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Mar  4 13:08:22 2011
 *
 * @brief  Implement an actionlib server to execute a
 * control_msgs::FollowJointTrajectoryAction. Follows the
 * given trajectory with the arm.
 *
 *
 */

#include "sr_mechanism_controllers/joint_trajectory_action_controller.hpp"
#include <std_msgs/Float64.h>
#include <boost/algorithm/string.hpp>
#include <string>

namespace shadowrobot
{
  JointTrajectoryActionController::JointTrajectoryActionController()
  {
    action_server = boost::shared_ptr<JTAS> (
        new JTAS("/r_arm_controller/joint_trajectory_action",
        boost::bind(&JointTrajectoryActionController::execute_trajectory, this, _1),
        false)
    );

    //Create a map of joint names to their command publishers
    //Hand joints
    //TODO: this could be read from the controller manager
    // rosservice call /controller_manager/list_controllers
    std::string hand_names[] = {
      "ffj0", "ffj3", "ffj4",
      "lfj0", "lfj3", "lfj4", "lfj5",
      "mfj0", "mfj3", "mfj4",
      "rfj0", "rfj3", "rfj4",
      "thj1", "thj2", "thj3", "thj4", "thj5",
      "wrj1", "wrj2"
    };
    for (size_t i = 0; i < 20; i++)
    {
      joint_pub[hand_names[i]] = nh.advertise<std_msgs::Float64>(
          "/sh_"+hand_names[i]+"_mixed_position_velocity_controller/command", 2);

      joint_pub[ boost::to_upper_copy(hand_names[i]) ] = nh.advertise<std_msgs::Float64>(
          "/sh_"+hand_names[i]+"_mixed_position_velocity_controller/command", 2);
    }

    //Arm joints
    std::string arm_names[] = {"sr", "ss", "es", "er"};
    for (size_t i = 0; i < 4; i++)
    {
      joint_pub[arm_names[i]] = nh.advertise<std_msgs::Float64>(
          "/sa_"+arm_names[i]+"_position_controller/command", 2);
    }

    //Arm joints: 2 naming conventions
    std::string arm_names_2[] = {"ShoulderJRotate", "ShoulderJSwing", "ElbowJSwing", "ElbowJRotate"};
    for (size_t i = 0; i < 4; i++)
    {
      joint_pub[arm_names_2[i]] = nh.advertise<std_msgs::Float64>(
          "/sa_"+arm_names[i]+"_position_controller/command", 2);
    }

    ROS_DEBUG("Starting JointTrajectoryActionController server");
    action_server->start();
  }

  void JointTrajectoryActionController::execute_trajectory(
      const control_msgs::FollowJointTrajectoryGoalConstPtr& goal
  ){
    bool success = true;
    std::vector<std::string> joint_names = goal->trajectory.joint_names;
    JointTrajectoryPointVec trajectory_points = goal->trajectory.points;
    trajectory_msgs::JointTrajectoryPoint trajectory_step;

    // TODO - We should probably be looking at goal->trajectory.header.stamp to
    // work out what time to start the action.
    //std::cout << goal->trajectory.header.stamp << " - " << ros::Time::now() << std::endl;

    //loop through the steps
    ros::Duration sleeping_time(0.0), last_time(0.0);
    for(size_t index_step = 0; index_step < trajectory_points.size(); ++index_step)
    {
      trajectory_step = trajectory_points[index_step];

      //check if preempted (cancelled), bail out if we are
      if (action_server->isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Joint Trajectory Action Preempted");
        action_server->setPreempted();
        success = false;
        break;
      }

      //send out the positions for this step to the joints
      for( size_t i = 0; i < joint_names.size(); i++ )
      {
        ROS_DEBUG_STREAM("trajectory: " << joint_names[i] << " " << trajectory_step.positions[i] << " / sleep: " << sleeping_time.toSec() );
        ros::Publisher pub = joint_pub[joint_names[i]];
        std_msgs::Float64 msg;
        msg.data = trajectory_step.positions[i];
        pub.publish(msg);
      }
      // Wait until this step is supposed to be completed.
      // TODO: This assumes that the movement will be instant! We should really
      // be working out how long the movement will take?
      sleeping_time =  trajectory_step.time_from_start - last_time;
      sleeping_time.sleep();
      last_time = trajectory_step.time_from_start;
    }

    //send the result back
    control_msgs::FollowJointTrajectoryResult joint_trajectory_result;
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
// vim: sw=2:ts=2
