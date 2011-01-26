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
  SrMoveArmSimpleAction::SrMoveArmSimpleAction()
  {
    action_server.reset(new actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction>("/arm/move", boost::bind(&SrMoveArmSimpleAction::execute, this, _1)));
  }

  SrMoveArmSimpleAction::execute(const move_arm_msgs::MoveArmGoalConstPtr& Goal)
  {
    ROS_ERROR("not implemented yet");
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
