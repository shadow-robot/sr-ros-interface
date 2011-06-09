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
  SrHandLib::SrHandLib(std::vector<std::string> joint_names,
                       std::vector<int> motor_ids,
                       std::vector<std::vector<int> > joint_ids,
                       std::vector<pr2_hardware_interface::Actuator*> actuators) :
    SrRobotLib(joint_names, motor_ids, joint_ids, actuators)
  {
    initialize_map(joint_names, motor_ids, joint_ids, actuators);
  }

  SrHandLib::~SrHandLib()
  {
    BOOST_FOREACH( shadow_joints::JointsMap::value_type &i, joints_map )
      delete i.second->motor->actuator;
  }

  void SrHandLib::initialize_map(std::vector<std::string> joint_names,
                                 std::vector<int> motor_ids,
                                 std::vector<std::vector<int> > joint_ids,
                                 std::vector<pr2_hardware_interface::Actuator*> actuators)
  {
    joints_map_mutex.lock();

    for(unsigned int index = 0; index < joint_names.size(); ++index)
    {
      boost::shared_ptr<shadow_joints::Joint> joint = boost::shared_ptr<shadow_joints::Joint>( new shadow_joints::Joint() );
      boost::shared_ptr<shadow_joints::Motor> motor = boost::shared_ptr<shadow_joints::Motor> ( new shadow_joints::Motor() );

      joint->joint_ids = joint_ids[index];
      motor->motor_id = motor_ids[index];

      motor->actuator = actuators[index];

      //TODO: check if the joint has a motor associated or not
      joint->has_motor = true;
      joint->motor     = motor;

      joints_map[ joint_names[index] ] = joint;
    }
    joints_map_mutex.unlock();
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
