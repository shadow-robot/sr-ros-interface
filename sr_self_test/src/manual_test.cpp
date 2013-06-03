/**
 * @file   manual_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Apr 16 06:03:09 2013
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
 * @brief We want to be able to run some tests with the user input (ask the
 *        user to choose whether the test failed or passed). This can be used
 *        for example for visually checking the calibration or pressing the
 *        tactiles manually.
 *
 *
 */

#include "sr_self_test/manual_test.hpp"
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <sr_robot_msgs/ManualSelfTest.h>

namespace shadow_robot
{
  ManualTests::ManualTests( std::string message, int id )
    : nh_("~")
  {
    message_ = message;
    id_ = id;

    user_input_client_ = nh_.serviceClient<sr_robot_msgs::ManualSelfTest>("manual_self_tests");
  }

  void ManualTests::run_manual_tests(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    user_input_client_.waitForExistence();

    sr_robot_msgs::ManualSelfTest srv;
    srv.request.message = message_;

    user_input_client_.call(srv);

    if( srv.response.ok )
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
    else
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Test failed: " + srv.response.message);
    }
  }
} //end namespace

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

