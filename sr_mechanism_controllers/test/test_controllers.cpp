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
#include "sr_mechanism_controllers/test/test_controllers.hpp"

TestControllers::TestControllers()
{
}

TestControllers::~TestControllers()
{
}

void TestControllers::init()
{
  hw = boost::shared_ptr<pr2_hardware_interface::HardwareInterface>( new pr2_hardware_interface::HardwareInterface() );

  //add a fake FFJ3 actuator
  actuator = boost::shared_ptr<pr2_hardware_interface::Actuator>( new pr2_hardware_interface::Actuator("FFJ3") );
  actuator->state_.is_enabled_ = true;
  hw->addActuator( actuator.get() );

  robot = boost::shared_ptr<pr2_mechanism_model::Robot>( new pr2_mechanism_model::Robot( hw.get()) );

  model = boost::shared_ptr<TiXmlDocument>( new TiXmlDocument() );
  bool loadOkay = model->LoadFile("/code/Projects/ROS_interfaces/sr-ros-interface/palm_edc/shadow_robot/sr_hand/model/robots/shadowhand_motor.urdf");

  if ( loadOkay )
  {
    robot->initXml( model->RootElement() );

    robot_state = boost::shared_ptr<pr2_mechanism_model::RobotState>( new pr2_mechanism_model::RobotState(robot.get()) );

    joint_state = robot_state->getJointState("FFJ3");
    joint_state->calibrated_ = true;

    init_controller();
    controller->starting();

    //initialize the controller:
    joint_state->position_ = 0.0;
    controller->update();
  }
  else
  {
    ROS_ERROR("Failed to load the model.");
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
