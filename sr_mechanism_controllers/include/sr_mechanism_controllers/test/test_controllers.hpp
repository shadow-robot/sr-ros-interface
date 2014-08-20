/**
 * @file   srh_joint_velocity_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Aug 17 12:32:01 2011
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
 * @brief  Follows a position target. The position demand is converted into a force
 * demand by a PID loop.
 *
 */

#ifndef _SR_TEST_CONTROLLERS_HPP_
#define _SR_TEST_CONTROLLERS_HPP_

#include "sr_mechanism_controllers/sr_controller.hpp"
#include "sr_mechanism_model/simple_transmission.hpp"

#include <control_toolbox/pid.h>
#include <tinyxml.h>
#include <sr_hardware_interface/sr_actuator.hpp>

class TestControllers
{
public:
  TestControllers();
  virtual ~TestControllers()
  {
    delete controller;
    delete hw;
    delete robot_state;
    delete model;
    delete joint_state;
  }

  void init();

  virtual void init_controller() = 0;
  virtual double compute_output(double input, double current_position) = 0;

  controller::SrController *controller;
  hardware_interface::HardwareInterface *hw;
  ros_ethercat_model::RobotState *robot_state;
  TiXmlDocument *model;
  sr_mechanism_model::SimpleTransmission transmission;
  ros_ethercat_model::JointState* joint_state;
};


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
