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
#include "sr_edc_mechanism_controllers/srh_joint_position_controller.hpp"
#include <boost/smart_ptr.hpp>
#include <gtest/gtest.h>

#include <control_toolbox/pid.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/robot.h>
#include <tinyxml/tinyxml.h>

using namespace controller;

class TestJointPositionController
{
public:
  boost::shared_ptr<SrhJointPositionController> controller;
  boost::shared_ptr<pr2_hardware_interface::HardwareInterface> hw;
  boost::shared_ptr<pr2_mechanism_model::Robot> robot;
  boost::shared_ptr<pr2_mechanism_model::RobotState> robot_state;
  boost::shared_ptr<TiXmlDocument> model;
  boost::shared_ptr<pr2_hardware_interface::Actuator> actuator;
  control_toolbox::Pid pid;

  pr2_mechanism_model::JointState* joint_state;

  TestJointPositionController()
  {
    controller = boost::shared_ptr<SrhJointPositionController>( new SrhJointPositionController() );

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

      controller->init(robot_state.get(), "FFJ3", pid);
    }
    else
    {
      ROS_ERROR("Failed to load the model.");
    }
  }

  ~TestJointPositionController()
  {}

  double compute_output(double input)
  {
    joint_state->position_ = 0.0;
    controller->setCommand( input );

    return joint_state->commanded_effort_;
  }
};

TEST(SrhJointPositionController, TestP)
{
  boost::shared_ptr<TestJointPositionController> test_jpc;
  test_jpc = boost::shared_ptr<TestJointPositionController>( new TestJointPositionController() );

  double ctrl_output = test_jpc->compute_output( 1.0 );

  EXPECT_EQ(ctrl_output, 1.0);
}



/////////////////////
//     MAIN       //
///////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_test");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
