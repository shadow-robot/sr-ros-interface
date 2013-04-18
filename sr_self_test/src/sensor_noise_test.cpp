/**
 * @file   sensor_noise_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Apr 18 06:33:35 2013
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
 * @brief We use this class to parse the joint states while the hand is immobile
 *        and check the noise of the sensor.
 *
 *
 */

#include "sr_self_test/sensor_noise_test.hpp"

namespace shadow_robot
{
  SensorNoiseTest::SensorNoiseTest()
  {
    joint_states_sub_ = nh_.subscribe("joint_states", 50, &SensorNoiseTest::joint_states_cb_, this);
  }

  void SensorNoiseTest::test_sensor_noise(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "TODO, implement.");
  }

  void SensorNoiseTest::joint_states_cb_(const sensor_msgs::JointState::ConstPtr& msg)
  {
    ROS_ERROR("RECEIVED");
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

