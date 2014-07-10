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

#include <math.h>
#include <pluginlib/class_list_macros.h>
#include "ros_ethercat_model/robot_state.hpp"
#include "sr_mechanism_model/simple_transmission.hpp"

#include <sr_hardware_interface/sr_actuator.hpp>

using namespace ros_ethercat_model;
using namespace std;
using namespace sr_actuator;

PLUGINLIB_EXPORT_CLASS(sr_mechanism_model::SimpleTransmission, Transmission)

namespace sr_mechanism_model
{

bool SimpleTransmission::initXml(TiXmlElement *elt, RobotState *robot)
{
  if (!ros_ethercat_model::Transmission::initXml(elt, robot))
    return false;

  TiXmlElement *ael = elt->FirstChildElement("actuator");
  string actuator_name = ael ? ael->Attribute("name") : "";
  Actuator *a = new SrMotorActuator();
  if (actuator_name.empty() || !a)
  {
    ROS_ERROR_STREAM("SimpleTransmission could not find actuator named : " << actuator_name);
    return false;
  }
  robot->actuators_.insert(actuator_name, a);
  a->command_.enable_ = true;
  actuator_names_.push_back(actuator_name);
  return true;
}

void SimpleTransmission::propagatePosition(vector<Actuator*>& as, vector<JointState*>& js)
{
  ROS_DEBUG(" propagate position");
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == 1);

  const ActuatorState &state = as[0]->state_;
  js[0]->position_ = state.position_;
  js[0]->velocity_ = state.velocity_;
  js[0]->measured_effort_ = state.last_measured_effort_;

  ROS_DEBUG("end propagate position");
}

void SimpleTransmission::propagateEffort(vector<JointState*>& js, vector<Actuator*>& as)
{
  ROS_DEBUG(" propagate effort");
  ROS_ASSERT(as.size() == 1);
  ROS_ASSERT(js.size() == 1);

  ActuatorCommand &command = as[0]->command_;
  command.enable_ = true;
  command.effort_ = js[0]->commanded_effort_;

  ROS_DEBUG("end propagate effort");
}

} //end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */
