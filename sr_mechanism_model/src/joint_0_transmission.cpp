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


#include "sr_mechanism_model/joint_0_transmission.hpp"

using namespace ros_ethercat_model;
using namespace std;
using namespace sr_actuator;

PLUGINLIB_EXPORT_CLASS(sr_mechanism_model::J0Transmission, Transmission)

namespace sr_mechanism_model
{

bool J0Transmission::initXml(TiXmlElement *elt, RobotState *robot)
{
  if (!SimpleTransmission::initXml(elt, robot))
    return false;

  string joint2_name = joint_->joint_->name;
  joint2_name[joint2_name.size()-1] = '2';
  joint2_ = robot->getJointState(joint2_name);

  return true;
}

void J0Transmission::propagatePosition()
{
  //the size is either 2 or 0 when the joint hasn't been updated yet
  // (joint 0 is composed of the 2 calibrated values: joint 1 and joint 2)

  SrMotorActuator *act = static_cast<SrMotorActuator*>(actuator_);
  size_t size = act->motor_state_.calibrated_sensor_values_.size();
  if (size == 2)
  {
    ROS_DEBUG_STREAM("READING pos " << act->state_.position_
                     << " J1 " << act->motor_state_.calibrated_sensor_values_[0]
                     << " J2 " << act->motor_state_.calibrated_sensor_values_[1]);

    joint_->position_ = act->motor_state_.calibrated_sensor_values_[0];
    joint2_->position_ = act->motor_state_.calibrated_sensor_values_[1];

    joint_->velocity_ = act->state_.velocity_ / 2.0;
    joint2_->velocity_ = act->state_.velocity_ / 2.0;

    joint_->effort_ = act->state_.last_measured_effort_;
    joint2_->effort_ = act->state_.last_measured_effort_;
  }
  else if (size == 0)
  {
    ROS_DEBUG_STREAM("READING pos from Gazebo " << act->state_.position_
                     << " J1 " << act->state_.position_ / 2.0
                     << " J2 " << act->state_.position_ / 2.0);

    //TODO: use a real formula for the coupling??
    //GAZEBO
    joint_->position_ = act->state_.position_ / 2.0;
    joint2_->position_ = act->state_.position_ / 2.0;

    joint_->velocity_ = act->state_.velocity_ / 2.0;
    joint2_->velocity_ = act->state_.velocity_ / 2.0;

    joint_->effort_ = act->state_.last_measured_effort_ / 2.0;
    joint2_->effort_ = act->state_.last_measured_effort_ / 2.0;
  }
}

} //end namespace sr_mechanism_model

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
