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
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/GraspStatus.h>


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
    void execute(const object_manipulation_msgs::GraspHandPostureExecutionGoalConstPtr& Goal);

    boost::shared_ptr<actionlib::SimpleActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction> > action_server;
  };//end class
}//end workspace

#endif SR_MOVE_ARM_SIMPLE_ACTION_H

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
