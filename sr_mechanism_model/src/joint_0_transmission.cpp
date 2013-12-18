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

#include <math.h>
#include <pluginlib/class_list_macros.h>
#include "pr2_mechanism_model/robot.h"
#include "pr2_mechanism_model/simple_transmission.h"

#include <sr_hardware_interface/sr_actuator.hpp>

using namespace pr2_hardware_interface;

PLUGINLIB_EXPORT_CLASS(sr_mechanism_model::J0Transmission, pr2_mechanism_model::Transmission)

namespace sr_mechanism_model
{
  bool J0Transmission::initXml(TiXmlElement *elt, pr2_mechanism_model::Robot *robot)
  {
    const char *name = elt->Attribute("name");
    name_ = name ? name : "";

    TiXmlElement *jel = elt->FirstChildElement("joint1");
    init_joint(jel, robot);
    TiXmlElement *jel2 = elt->FirstChildElement("joint2");
    init_joint(jel2, robot);

    TiXmlElement *ael = elt->FirstChildElement("actuator");
    const char *actuator_name = ael ? ael->Attribute("name") : NULL;
    Actuator *a;
    if (!actuator_name || (a = robot->getActuator(actuator_name)) == NULL )
    {
      ROS_ERROR("J0Transmission could not find actuator named \"%s\"", actuator_name);
      return false;
    }
    a->command_.enable_ = true;
    actuator_names_.push_back(actuator_name);

    mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

    return true;
  }

  bool J0Transmission::initXml(TiXmlElement *elt)
  {
    const char *name = elt->Attribute("name");
    name_ = name ? name : "";

    TiXmlElement *jel = elt->FirstChildElement("joint1");
    init_joint(jel, NULL);
    TiXmlElement *jel2 = elt->FirstChildElement("joint2");
    init_joint(jel2, NULL);


    TiXmlElement *ael = elt->FirstChildElement("actuator");
    const char *actuator_name = ael ? ael->Attribute("name") : NULL;
    if (!actuator_name)
    {
      ROS_ERROR("J0Transmission could not find actuator named \"%s\"", actuator_name);
      return false;
    }
    actuator_names_.push_back(actuator_name);

    mechanical_reduction_ = atof(elt->FirstChildElement("mechanicalReduction")->GetText());

    return true;
  }

  bool J0Transmission::init_joint(TiXmlElement *jel, pr2_mechanism_model::Robot *robot)
  {
    const char *joint_name = jel ? jel->Attribute("name") : NULL;
    if (!joint_name)
    {
      ROS_ERROR("J0Transmission did not specify joint name");
      return false;
    }

    if( robot != NULL )
    {
      const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
      if (!joint)
      {
        ROS_ERROR("J0Transmission could not find joint named \"%s\"", joint_name);
        return false;
      }
    }
    joint_names_.push_back(joint_name);

    return true;
  }

  void J0Transmission::propagatePosition(
    std::vector<pr2_hardware_interface::Actuator*>& as, std::vector<pr2_mechanism_model::JointState*>& js)
  {
    ROS_DEBUG(" propagate position for j0");

    assert(as.size() == 1);
    assert(js.size() == 2);

    //the size is either 0 (when the joint hasn't been updated yet), either 2
    // (joint 0 is composed of the 2 calibrated values: joint 1 and joint 2)
    int size = static_cast<sr_actuator::SrActuator*>(as[0])->state_.calibrated_sensor_values_.size();
    if( size != 0)
    {
      if( size == 2 )
      {
        ROS_DEBUG_STREAM( "READING pos " << static_cast<sr_actuator::SrActuator*>(as[0])->state_.position_
                          << " J1 " << static_cast<sr_actuator::SrActuator*>(as[0])->state_.calibrated_sensor_values_[0]
                          << " J2 " << static_cast<sr_actuator::SrActuator*>(as[0])->state_.calibrated_sensor_values_[1] );

        js[0]->position_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.calibrated_sensor_values_[0];
        js[1]->position_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.calibrated_sensor_values_[1];

        js[0]->velocity_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.velocity_ / 2.0;
        js[1]->velocity_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.velocity_ / 2.0;

        js[0]->measured_effort_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.last_measured_effort_;
        js[1]->measured_effort_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.last_measured_effort_;
      }
    }
    else
    {
      ROS_DEBUG_STREAM( "READING pos from Gazebo " << static_cast<sr_actuator::SrActuator*>(as[0])->state_.position_
                        << " J1 " << static_cast<sr_actuator::SrActuator*>(as[0])->state_.position_ / 2.0
                        << " J2 " << static_cast<sr_actuator::SrActuator*>(as[0])->state_.position_ / 2.0 );

      //TODO: use a real formula for the coupling??
      //GAZEBO
      js[0]->position_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.position_ / 2.0;
      js[1]->position_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.position_ / 2.0;

      js[0]->velocity_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.velocity_ / 2.0;
      js[1]->velocity_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.velocity_ / 2.0;

      js[0]->measured_effort_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.last_measured_effort_ / 2.0;
      js[1]->measured_effort_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.last_measured_effort_ / 2.0;
    }

    ROS_DEBUG("end propagate position for j0");
  }

  void J0Transmission::propagatePositionBackwards(
    std::vector<pr2_mechanism_model::JointState*>& js, std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    ROS_DEBUG("propagate pos backward for j0");

    assert(as.size() == 1);
    assert(js.size() == 2);

    ROS_DEBUG_STREAM("  pos = " << js[0]->position_ << " + " << js[1]->position_ << " = " << static_cast<sr_actuator::SrActuator*>(as[0])->state_.position_);
    static_cast<sr_actuator::SrActuator*>(as[0])->state_.position_ = js[0]->position_ + js[1]->position_;
    static_cast<sr_actuator::SrActuator*>(as[0])->state_.velocity_ = js[0]->velocity_ + js[1]->velocity_;
    static_cast<sr_actuator::SrActuator*>(as[0])->state_.last_measured_effort_ = js[0]->measured_effort_ + js[1]->measured_effort_;

    // Update the timing (making sure it's initialized).
    if (! simulated_actuator_timestamp_initialized_)
    {
      // Set the time stamp to zero (it is measured relative to the start time).
      static_cast<sr_actuator::SrActuator*>(as[0])->state_.sample_timestamp_ = ros::Duration(0);

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
      static_cast<sr_actuator::SrActuator*>(as[0])->state_.sample_timestamp_ = ros::Time::now() - simulated_actuator_start_time_;
    }
    // Set the historical (double) timestamp accordingly.
    static_cast<sr_actuator::SrActuator*>(as[0])->state_.timestamp_ = static_cast<sr_actuator::SrActuator*>(as[0])->state_.sample_timestamp_.toSec();

    // simulate calibration sensors by filling out actuator states
    this->joint_calibration_simulator_.simulateJointCalibration(js[0],static_cast<sr_actuator::SrActuator*>(as[0]));

    ROS_DEBUG(" end propagate pos backward for j0");
  }

  void J0Transmission::propagateEffort(
    std::vector<pr2_mechanism_model::JointState*>& js, std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    ROS_DEBUG(" propagate effort for j0");

    assert(as.size() == 1);
    assert(js.size() == 2);
    static_cast<sr_actuator::SrActuator*>(as[0])->command_.enable_ = true;
    static_cast<sr_actuator::SrActuator*>(as[0])->command_.effort_ = (js[0]->commanded_effort_ + js[1]->commanded_effort_);

    ROS_DEBUG("end propagate effort for j0");
  }

  void J0Transmission::propagateEffortBackwards(
    std::vector<pr2_hardware_interface::Actuator*>& as, std::vector<pr2_mechanism_model::JointState*>& js)
  {
    ROS_DEBUG("propagate effort backward for j0");

    assert(as.size() == 1);
    assert(js.size() == 2);
    js[0]->commanded_effort_ = static_cast<sr_actuator::SrActuator*>(as[0])->command_.effort_;
    js[1]->commanded_effort_ = static_cast<sr_actuator::SrActuator*>(as[0])->command_.effort_;

    ROS_DEBUG("end propagate effort backward for j0");
  }

} //end namespace sr_mechanism_model

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
