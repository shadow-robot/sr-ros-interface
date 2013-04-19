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
  ManualTests::ManualTests()
    : nh_("~")
  {
    user_input_client_ = nh_.serviceClient<sr_robot_msgs::ManualSelfTest>("manual_self_tests");
  }

  void ManualTests::run_manual_tests(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    user_input_client_.waitForExistence();

    //Run Tactile test
    //The user needs to start rxplot for the tactiles
    // ask the user to press the tactiles
    sr_robot_msgs::ManualSelfTest tactile_srv;
    tactile_srv.request.message = "Please press on the tactile sensors one after the other. Check that they react using rxplot.";
    user_input_client_.call(tactile_srv);

    //Run Calibration test
    //The user needs to start rviz
    // ask the user to check the calibration visually
    sr_robot_msgs::ManualSelfTest calibration_srv;
    calibration_srv.request.message = "Please check that the positions of the joints in the 3d model of the hand (using rviz) match those in the real hand.";
    user_input_client_.call(calibration_srv);

    if( tactile_srv.response.ok && calibration_srv.response.ok )
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Tactile and calibrations are ok.");
      return;
    }

    if( !tactile_srv.response.ok )
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Tactile test failed: " + tactile_srv.response.message);
    }
    if( !calibration_srv.response.ok )
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Calibration test failed: " + calibration_srv.response.message);
    }
  }
} //end namespace

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

