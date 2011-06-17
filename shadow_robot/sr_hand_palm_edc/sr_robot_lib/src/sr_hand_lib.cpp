/**
 * @file   sr_hand_lib.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, <contact@shadowrobot.com>
 * @date   Fri Jun  3 13:05:10 2011
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
*
 * @brief This is a library for the etherCAT hand.
 * You can find it instantiated in the sr_edc_ethercat_drivers.
 *
 *
 */

#include "sr_robot_lib/sr_hand_lib.hpp"
#include <string>
#include <boost/foreach.hpp>

namespace shadow_robot
{
  SrHandLib::SrHandLib(std::vector<std::string> joint_names, std::vector<int> motor_ids,
                       std::vector<shadow_joints::JointToSensor> joint_to_sensors,
                       std::vector<pr2_hardware_interface::Actuator*> actuators,
                       shadow_joints::CalibrationMap calibration_map) :
    SrRobotLib(joint_names, motor_ids, joint_to_sensors, actuators, calibration_map)
  {
    this->calibration_map = calibration_map;

    initialize_maps(joint_names, motor_ids, joint_to_sensors, actuators);
  }

  SrHandLib::~SrHandLib()
  {
    boost::shared_ptr<shadow_joints::Joint> joint_tmp;
    BOOST_FOREACH( std::string joint_name, joints_map.keys() )
    {
      joint_tmp = joints_map.find(joint_name);
      delete joint_tmp->motor->actuator;
    }
  }

  void SrHandLib::initialize_maps(std::vector<std::string> joint_names, std::vector<int> motor_ids,
                                  std::vector<shadow_joints::JointToSensor> joint_to_sensors,
                                  std::vector<pr2_hardware_interface::Actuator*> actuators)
  {
    for(unsigned int index = 0; index < joint_names.size(); ++index)
    {
      boost::shared_ptr<shadow_joints::Joint> joint = boost::shared_ptr<shadow_joints::Joint>( new shadow_joints::Joint() );
      boost::shared_ptr<shadow_joints::Motor> motor = boost::shared_ptr<shadow_joints::Motor> ( new shadow_joints::Motor() );

      joint->joint_to_sensor = joint_to_sensors[index];

      if(motor_ids[index] == -1) //no motor associated to this joint
        joint->has_motor = false;
      else
        joint->has_motor = true;

      motor->motor_id = motor_ids[index];
      motor->actuator = actuators[index];
      joint->motor     = motor;

      joints_map.insert( joint_names[index], joint);
    }
  }

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
