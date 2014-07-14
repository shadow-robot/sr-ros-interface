/**
 * @file   joint_0_transmission_for_muscle.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
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

#include "sr_mechanism_model/joint_0_transmission_for_muscle.hpp"

using namespace ros_ethercat_model;
using namespace std;
using namespace sr_actuator;

PLUGINLIB_EXPORT_CLASS(sr_mechanism_model::J0TransmissionForMuscle, Transmission)

namespace sr_mechanism_model
{

bool J0TransmissionForMuscle::initXml(TiXmlElement *elt, RobotState *robot)
{
  if (!J0Transmission::initXml(elt, robot))
    return false;

  string act_name = actuator_->name_;
  delete actuator_; // That's a SrMotorActuator at this point
  actuator_ = new SrMuscleActuator();
  actuator_->name_ = act_name;
  actuator_->command_.enable_ = true;

  return true;
}

void J0TransmissionForMuscle::propagatePosition()
{
  //the size is either 2 or 0 when the joint hasn't been updated yet
  // (joint 0 is composed of the 2 calibrated values: joint 1 and joint 2)
  SrMuscleActuatorState &state = static_cast<SrMuscleActuator*>(actuator_)->state_;
  size_t size = state.calibrated_sensor_values_.size();
  if (size == 0)
  {
    ROS_DEBUG_STREAM("READING pos " << state.position_
                     << " J1 " << state.calibrated_sensor_values_[0]
                     << " J2 " << state.calibrated_sensor_values_[1]);

    joint_->position_ = state.calibrated_sensor_values_[0];
    joint2_->position_ = state.calibrated_sensor_values_[1];

    joint_->velocity_ = state.velocity_ / 2.0;
    joint2_->velocity_ = state.velocity_ / 2.0;

    // We don't want to define a modified version of JointState, as that would imply using a modified version
    // of robot_state.hpp, controller manager, ethercat_hardware and ros_etherCAT main loop
    // So we will encode the two uint16_t that contain the data from the muscle pressure sensors
    // into the double measured_effort_. (We don't have any measured effort in the muscle hand anyway).
    // Then in the joint controller we will decode that back into uint16_t.
    joint_->measured_effort_ = ((double) (state.pressure_[1]) * 0x10000) + (double) (state.pressure_[0]);
    joint2_->measured_effort_ = ((double) (state.pressure_[1]) * 0x10000) + (double) (state.pressure_[0]);
  }
  else if (size == 0)
  {
    ROS_DEBUG_STREAM("READING pos from Gazebo " << state.position_
                     << " J1 " << state.position_ / 2.0
                     << " J2 " << state.position_ / 2.0);

    //TODO: use a real formula for the coupling??
    //GAZEBO
    joint_->position_ = state.position_ / 2.0;
    joint2_->position_ = state.position_ / 2.0;

    joint_->velocity_ = state.velocity_ / 2.0;
    joint2_->velocity_ = state.velocity_ / 2.0;

    joint_->measured_effort_ = state.last_measured_effort_ / 2.0;
    joint2_->measured_effort_ = state.last_measured_effort_ / 2.0;
  }
}

void J0TransmissionForMuscle::propagateEffort()
{
  SrMuscleActuatorCommand &command = static_cast<SrMuscleActuator*>(actuator_)->command_;
  command.enable_ = true;

  // We don't want to define a modified version of JointState, as that would imply using a modified version
  // of robot_state.hpp, controller manager, ethercat_hardware and ros_etherCAT main loop
  // So the controller encodes the two int16 that contain the valve commands into the double effort_.
  // (We don't have any real commanded_effort_ in the muscle hand anyway).
  // Here we decode them back into two int16_t.
  double valve_0 = fmod(joint_->commanded_effort_, 0x10);
  int8_t valve_0_tmp = (int8_t) (valve_0 + 0.5);
  if (valve_0_tmp >= 8)
  {
    valve_0_tmp -= 8;
    valve_0_tmp *= (-1);
  }

  int8_t valve_1_tmp = (int8_t) (((fmod(joint_->commanded_effort_, 0x100) - valve_0) / 0x10) + 0.5);
  if (valve_1_tmp >= 8)
  {
    valve_1_tmp -= 8;
    valve_1_tmp *= (-1);
  }

  command.valve_[0] = valve_0_tmp;
  command.valve_[1] = valve_1_tmp;
}

} //end namespace sr_mechanism_model

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
