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
#include <boost/smart_ptr.hpp>

#include <control_toolbox/pid.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/robot.h>
#include <tinyxml.h>
#include <sr_hardware_interface/sr_actuator.hpp>

class TestControllers
{
public:
  TestControllers();
  virtual ~TestControllers();

  void init();

  virtual void init_controller() = 0;
  virtual double compute_output(double input, double current_position) = 0;

  boost::shared_ptr<controller::SrController> controller;
  boost::shared_ptr<pr2_hardware_interface::HardwareInterface> hw;
  boost::shared_ptr<pr2_mechanism_model::Robot> robot;
  boost::shared_ptr<pr2_mechanism_model::RobotState> robot_state;
  boost::shared_ptr<TiXmlDocument> model;
  boost::shared_ptr<sr_actuator::SrMotorActuator> actuator;
  pr2_mechanism_model::JointState* joint_state;
};


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
