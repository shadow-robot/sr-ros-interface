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
#include "pr2_mechanism_model/robot.h"
#include "sr_mechanism_model/simple_transmission_for_muscle.hpp"

#include <sr_hardware_interface/sr_actuator.hpp>

using namespace pr2_hardware_interface;

PLUGINLIB_EXPORT_CLASS(sr_mechanism_model::SimpleTransmissionForMuscle, pr2_mechanism_model::Transmission)

namespace sr_mechanism_model
{
  bool SimpleTransmissionForMuscle::initXml(TiXmlElement *elt, pr2_mechanism_model::Robot *robot)
  {
    const char *name = elt->Attribute("name");
    name_ = name ? name : "";

    TiXmlElement *jel = elt->FirstChildElement("joint");
    const char *joint_name = jel ? jel->Attribute("name") : NULL;
    if (!joint_name)
    {
      ROS_ERROR("SimpleTransmissionForMuscle did not specify joint name");
      return false;
    }

    const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
    if (!joint)
    {
      ROS_ERROR("SimpleTransmissionForMuscle could not find joint named \"%s\"", joint_name);
      return false;
    }
    joint_names_.push_back(joint_name);

    TiXmlElement *ael = elt->FirstChildElement("actuator");
    const char *actuator_name = ael ? ael->Attribute("name") : NULL;
    pr2_hardware_interface::Actuator *a;
    if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
    {
      ROS_ERROR("SimpleTransmissionForMuscle could not find actuator named \"%s\"", actuator_name);
      return false;
    }
    a->command_.enable_ = true;
    actuator_names_.push_back(actuator_name);

    mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

    return true;
  }

  bool SimpleTransmissionForMuscle::initXml(TiXmlElement *elt)
  {
    const char *name = elt->Attribute("name");
    name_ = name ? name : "";

    TiXmlElement *jel = elt->FirstChildElement("joint");
    const char *joint_name = jel ? jel->Attribute("name") : NULL;
    if (!joint_name)
    {
      ROS_ERROR("SimpleTransmissionForMuscle did not specify joint name");
      return false;
    }
    joint_names_.push_back(joint_name);

    TiXmlElement *ael = elt->FirstChildElement("actuator");
    const char *actuator_name = ael ? ael->Attribute("name") : NULL;
    if (!actuator_name)
    {
      ROS_ERROR("SimpleTransmissionForMuscle could not find actuator named \"%s\"", actuator_name);
      return false;
    }
    actuator_names_.push_back(actuator_name);

    mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

    return true;
  }

  void SimpleTransmissionForMuscle::propagatePosition(
    std::vector<pr2_hardware_interface::Actuator*>& as, std::vector<pr2_mechanism_model::JointState*>& js)
  {
    ROS_DEBUG(" propagate position");

    assert(as.size() == 1);
    assert(js.size() == 1);
    js[0]->position_ = static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.position_;
    js[0]->velocity_ = static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.velocity_;
    //We don't want to define a modified version of JointState, as that would imply using a modified version of robot.h, controller manager,
    //ethercat_hardware and pr2_etherCAT main loop
    // So we will encode the two uint16 that contain the data from the muscle pressure sensors into the double measured_effort_. (We don't
    // have any measured effort in the muscle hand anyway).
    // Then in the joint controller we will decode that back into uint16.
    js[0]->measured_effort_ = (static_cast<double>(static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.pressure_[1]) * 0x10000) +
                              static_cast<double>(static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.pressure_[0]);

    ROS_DEBUG("end propagate position");
  }

  void SimpleTransmissionForMuscle::propagatePositionBackwards(
    std::vector<pr2_mechanism_model::JointState*>& js, std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    ROS_DEBUG(" propagate position bw");

    assert(as.size() == 1);
    assert(js.size() == 1);
    static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.position_ = js[0]->position_;
    static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.velocity_ = js[0]->velocity_;
    static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.last_measured_effort_ = js[0]->measured_effort_;

    // Update the timing (making sure it's initialized).
    if (! simulated_actuator_timestamp_initialized_)
    {
      // Set the time stamp to zero (it is measured relative to the start time).
      static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.sample_timestamp_ = ros::Duration(0);

      // Try to set the start time.  Only then do we claim initialized.
      if (ros::isStarted())
      {
        simulated_actuator_start_time_ = ros::Time::now();
        simulated_actuator_timestamp_initialized_ = true;
      }
    }
    else
    {
      // Measure the time stamp relative to the start time.
      static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.sample_timestamp_ = ros::Time::now() - simulated_actuator_start_time_;
    }
    // Set the historical (double) timestamp accordingly.
    static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.timestamp_ = static_cast<sr_actuator::SrMuscleActuator*>(as[0])->state_.sample_timestamp_.toSec();

    // simulate calibration sensors by filling out actuator states
    this->joint_calibration_simulator_.simulateJointCalibration(js[0],static_cast<sr_actuator::SrMuscleActuator*>(as[0]));

    ROS_DEBUG(" end propagate position bw");
  }

  void SimpleTransmissionForMuscle::propagateEffort(
    std::vector<pr2_mechanism_model::JointState*>& js, std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    ROS_DEBUG(" propagate effort");

    assert(as.size() == 1);
    assert(js.size() == 1);
    static_cast<sr_actuator::SrMuscleActuator*>(as[0])->command_.enable_ = true;
    //We don't want to define a modified version of JointState, as that would imply using a modified version of robot.h, controller manager,
    //ethercat_hardware and pr2_etherCAT main loop
    // So the controller encodes the two int16 that contain the valve commands into the double effort_. (We don't
    // have any real commanded_effort_ in the muscle hand anyway).
    // Here we decode them back into two int16.
    double valve_0 = fmod(js[0]->commanded_effort_, 0x10);
    int8_t valve_0_tmp = static_cast<int8_t>(valve_0 + 0.5);
    if (valve_0_tmp >= 8)
    {
      valve_0_tmp -= 8;
      valve_0_tmp *= (-1);
    }

    int8_t valve_1_tmp = static_cast<int8_t>(((fmod(js[0]->commanded_effort_, 0x100) - valve_0) / 0x10) + 0.5);
    if (valve_1_tmp >= 8)
    {
      valve_1_tmp -= 8;
      valve_1_tmp *= (-1);
    }

    static_cast<sr_actuator::SrMuscleActuator*>(as[0])->command_.valve_[0] = valve_0_tmp;
    static_cast<sr_actuator::SrMuscleActuator*>(as[0])->command_.valve_[1] = valve_1_tmp;

    ROS_DEBUG("end propagate effort");
  }

  void SimpleTransmissionForMuscle::propagateEffortBackwards(
    std::vector<pr2_hardware_interface::Actuator*>& as, std::vector<pr2_mechanism_model::JointState*>& js)
  {
    ROS_DEBUG(" propagate effort bw");

    assert(as.size() == 1);
    assert(js.size() == 1);
    js[0]->commanded_effort_ = static_cast<sr_actuator::SrMuscleActuator*>(as[0])->command_.effort_;

    ROS_DEBUG("end propagate effort bw");
  }

} //end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
