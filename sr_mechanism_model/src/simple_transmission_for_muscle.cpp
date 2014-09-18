/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 * Author: Stuart Glaser
 *
 * modified by Ugo Cupcic
 */

#include "sr_mechanism_model/simple_transmission_for_muscle.hpp"

using namespace ros_ethercat_model;
using namespace std;
using namespace sr_actuator;

PLUGINLIB_EXPORT_CLASS(sr_mechanism_model::SimpleTransmissionForMuscle, Transmission)

namespace sr_mechanism_model
{

bool SimpleTransmissionForMuscle::initXml(TiXmlElement *elt, RobotState *robot)
{
  if (!SimpleTransmission::Transmission::initXml(elt, robot))
    return false;

  string act_name = actuator_->name_;
  delete actuator_; // That's a SrMotorActuator at this point
  actuator_ = new SrMuscleActuator();
  actuator_->name_ = act_name;
  actuator_->command_.enable_ = true;

  return true;
}

void SimpleTransmissionForMuscle::propagatePosition()
{
  SrMuscleActuator *act = static_cast<SrMuscleActuator*>(actuator_);
  joint_->position_ = act->state_.position_;
  joint_->velocity_ = act->state_.velocity_;

  // We don't want to define a modified version of JointState, as that would imply using a modified version
  // of robot_state.hpp, controller manager, ethercat_hardware and ros_etherCAT main loop
  // So we will encode the two uint16_t that contain the data from the muscle pressure sensors
  // into the double effort_. (We don't have any measured effort in the muscle hand anyway).
  // Then in the joint controller we will decode that back into uint16_t.
  joint_->effort_ = ((double) (act->muscle_state_.pressure_[1]) * 0x10000)
                           +  (double) (act->muscle_state_.pressure_[0]);
}

void SimpleTransmissionForMuscle::propagateEffort()
{
  SrMuscleActuator *act = static_cast<SrMuscleActuator*>(actuator_);
  act->command_.enable_ = true;

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

  act->muscle_command_.valve_[0] = valve_0_tmp;
  act->muscle_command_.valve_[1] = valve_1_tmp;
}

} //end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
