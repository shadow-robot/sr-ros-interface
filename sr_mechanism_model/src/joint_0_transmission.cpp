/**
 * @file   joint_0_transmission.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Jun 28 11:35:05 2011
 *
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
 * @brief This is the implementation of the transmission for the joint 0s.
 * We need a specific transmission which takes into account that 2 joints
 * are actuated with only one actuator.
 *
 *
 */


#include <math.h>
#include <pluginlib/class_list_macros.h>
#include "ros_ethercat_model/robot_state.hpp"
#include "sr_mechanism_model/joint_0_transmission.hpp"

#include <sr_hardware_interface/sr_actuator.hpp>

using namespace ros_ethercat_model;
using namespace std;
using namespace sr_actuator;

PLUGINLIB_EXPORT_CLASS(sr_mechanism_model::J0Transmission, Transmission)

namespace sr_mechanism_model
{

void J0Transmission::propagatePosition(Actuator *as, vector<JointState*> &js)
{
  ROS_DEBUG(" propagate position for j0");
  ROS_ASSERT(js.size() == 2);

  //the size is either 2 or 0 when the joint hasn't been updated yet
  // (joint 0 is composed of the 2 calibrated values: joint 1 and joint 2)

  const SrMotorActuatorState &state = static_cast<SrMotorActuator*>(as)->state_;
  size_t size = state.calibrated_sensor_values_.size();
  if (size == 2)
  {
    ROS_DEBUG_STREAM("READING pos " << state.position_
                     << " J1 " << state.calibrated_sensor_values_[0]
                     << " J2 " << state.calibrated_sensor_values_[1]);

    js[0]->position_ = state.calibrated_sensor_values_[0];
    js[1]->position_ = state.calibrated_sensor_values_[1];

    js[0]->velocity_ = state.velocity_ / 2.0;
    js[1]->velocity_ = state.velocity_ / 2.0;

    js[0]->measured_effort_ = state.last_measured_effort_;
    js[1]->measured_effort_ = state.last_measured_effort_;
  }
  else if (size == 0)
  {
    ROS_DEBUG_STREAM("READING pos from Gazebo " << state.position_
                     << " J1 " << state.position_ / 2.0
                     << " J2 " << state.position_ / 2.0);

    //TODO: use a real formula for the coupling??
    //GAZEBO
    js[0]->position_ = state.position_ / 2.0;
    js[1]->position_ = state.position_ / 2.0;

    js[0]->velocity_ = state.velocity_ / 2.0;
    js[1]->velocity_ = state.velocity_ / 2.0;

    js[0]->measured_effort_ = state.last_measured_effort_ / 2.0;
    js[1]->measured_effort_ = state.last_measured_effort_ / 2.0;
  }

  ROS_DEBUG("end propagate position for j0");
}

void J0Transmission::propagateEffort(vector<JointState*> &js, Actuator *as)
{
  ROS_DEBUG(" propagate effort for j0");
  ROS_ASSERT(js.size() == 2);

  ActuatorCommand &command = as->command_;
  command.enable_ = true;
  command.effort_ = (js[0]->commanded_effort_ + js[1]->commanded_effort_);

  ROS_DEBUG("end propagate effort for j0");
}

} //end namespace sr_mechanism_model

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
