/**
 * @file   motor_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Apr 22 05:47:43 2013
 *
 * Copyright 2011 Shadow Robot Company Ltd.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * @brief Run different tests on the motors (apply PWM offset
 *        and measure current, test strain gauges, etc.. )
 *
 *
 */

#include "sr_self_test/motor_test.hpp"

namespace shadow_robot
{
  MotorTest::MotorTest(self_test::TestRunner* test_runner,
                       std::string joint_name,
                       shadowrobot::HandCommander* hand_commander)
    : test_runner_(test_runner), joint_name_(joint_name), hand_commander_(hand_commander)
  {
    test_runner_->add("Test motor ["+joint_name_+"]", this, &MotorTest::run_test);

    // @todo: subscribe to relevant topic and services
    change_ctrl_type_client_ = nh_.serviceClient<sr_robot_msgs::ChangeControlType>("change_control_type");
  }

  void MotorTest::run_test(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // @todo: go to PWM mode
    switch_to_PWM_();

    // @todo: apply PWM and record data

    // @todo: analyse data

    // @todo: reset to previous control mode
    switch_to_PWM_(true);

    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "TODO: implement this");
  }

  bool MotorTest::switch_to_PWM_(bool switch_back)
  {
    sr_robot_msgs::ChangeControlType srv;

    if( !switch_back )
      srv.request.control_type.control_type = sr_robot_msgs::ControlType::PWM;
    else
    {
      //@todo: how can we know which type of control it was before?
      srv.request.control_type.control_type = sr_robot_msgs::ControlType::TORQUE;
    }

    //fist we change to the correct control type
    if( change_ctrl_type_client_.call(srv) )
    {
      //now we load the proper controllers.
    }
    else
    {
      if(switch_back)
        ROS_ERROR("Failed to switch back to previous controllers.");
      else
        ROS_ERROR("Failed to switch to PWM control");

      return false;
    }
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

