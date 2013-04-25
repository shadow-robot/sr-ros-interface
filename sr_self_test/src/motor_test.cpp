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
  }

  void MotorTest::run_test(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    //go to effort mode
    if( !switch_to_effort_() )
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to switch to effort mode - aborting test.");
      return;
    }

    // @todo: apply effort and record data

    // @todo: analyse data

    //reset to previous control mode
    if( !switch_to_effort_(true) )
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Failed to switch back to previous controllers - test failed.");
      return;
    }

    //test succeeded
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Test passed.");
  }

  bool MotorTest::switch_to_effort_(bool switch_back)
  {

    //list the current controllers
    //pr2_mechanism_msgs::ListControllers list_ctrl;
    //now we load the proper controllers.

    return true;
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

