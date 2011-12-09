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
#include <sr_robot_msgs/sendupdate.h>
#include <sr_robot_msgs/is_hand_occupied.h>

#include <actionlib/server/simple_action_server.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/GraspStatus.h>
#include <object_manipulation_msgs/ManipulationResult.h>

#include <trajectory_msgs/JointTrajectory.h>

using namespace ros;

namespace shadowrobot
{
  class SrHandPostureExecutionSimpleAction
  {
  public:
    SrHandPostureExecutionSimpleAction();
    ~SrHandPostureExecutionSimpleAction();

  protected:
    NodeHandle nh, nh_tilde;
    Publisher pub;
    void execute(const object_manipulation_msgs::GraspHandPostureExecutionGoalConstPtr& goal);
    bool getStatusCallback(object_manipulation_msgs::GraspStatus::Request &request,
                           object_manipulation_msgs::GraspStatus::Response &response);

    boost::shared_ptr<actionlib::SimpleActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction> > action_server;

    Publisher sr_hand_target_pub;
    ServiceServer get_status_server;
    ServiceClient is_hand_occupied_client;
    sr_robot_msgs::sendupdate sendupdate_msg;
    std::vector<sr_robot_msgs::joint> joint_vector;

    bool hand_occupied;
  };//end class
}//end workspace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif SR_MOVE_ARM_SIMPLE_ACTION_H
