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
 * @brief
 *
 *
 */

#include "sr_self_test/manual_test.hpp"
#include <diagnostic_msgs/DiagnosticStatus.h>

namespace shadow_robot
{
  ManualTests::ManualTests()
  {
    //Subscribes to tactiles
    ros::Subscriber tactile_ps3_sub_;
    ros::Subscriber tactile_pst_sub_;
  }

  void ManualTests::run_manual_tests(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Implement test");
  }

  void ManualTests::tactile_ps3_cb_(sr_robot_msgs::BiotacAllConstPtr& msg)
  {}

  void ManualTests::tactile_pst_cb_(sr_robot_msgs::ShadowPSTConstPtr& msg)
  {}
} //end namespace

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

