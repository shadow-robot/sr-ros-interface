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

#include "sr_mechanism_controllers/srh_joint_position_controller.hpp"

#include <gtest/gtest.h>

using namespace controller;

class TestJointPositionController : public TestControllers
{
public:
  boost::shared_ptr<control_toolbox::Pid> pid;

  TestJointPositionController(boost::shared_ptr<control_toolbox::Pid> pid) :
    TestControllers()
  {
    this->pid = pid;
    controller = boost::shared_ptr<SrhJointPositionController>( new SrhJointPositionController() );

    init();
  }

  virtual ~TestJointPositionController()
  {}

  void init_controller()
  {
    controller::SrController* control_tmp = controller.get();
    controller::SrhJointPositionController* sr_control_tmp = dynamic_cast< controller::SrhJointPositionController* >( control_tmp );
    sr_control_tmp->init(robot_state.get(), "FFJ3", pid);
  }

  double compute_output(double input, double current_position)
  {
    hw->current_time_ = ros::Time::now();
    joint_state->position_ = current_position;
    joint_state->commanded_position_ = input;

    controller->update();

    return joint_state->commanded_effort_;
  }
};

TEST(SrhJointPositionController, TestPID)
{
  //TESTING A PURE P CONTROLLER
  boost::shared_ptr<control_toolbox::Pid> pid;
  pid = boost::shared_ptr<control_toolbox::Pid>( new control_toolbox::Pid(1.0, 0.0, 0.0, 0.0, 0.0) );

  boost::shared_ptr<TestJointPositionController> test_jpc;
  test_jpc = boost::shared_ptr<TestJointPositionController>( new TestJointPositionController( pid ) );

  const unsigned int nb_values = 7;

  bool with_friction_compensation = false;
  if( ros::param::has("with_friction_compensation") )
  {
    int wfc;
    ros::param::get("with_friction_compensation", wfc);
    if( wfc == 1)
      with_friction_compensation = true;
    else
      with_friction_compensation = false;
  }

  const double values[nb_values] = {-123.123, -1.0, -0.5, 0.0, 0.5, 1.0, 456.456};
  const double expected_values_no_fc[nb_values] = {-123.123, -1.0, -0.5, 0.0, 0.5, 1.0, 456.456};
  const double expected_values_with_fc[nb_values] = {-323.123, -1.0, -0.5, 0.0, 0.5, 1.0, 656.456};

  ros::Duration pause(0.01);
  double ctrl_output = 0.0;
  for(unsigned int i = 0; i < nb_values; ++i)
  {
    ROS_INFO_STREAM("Sending demand: "<<values[i]);
    pause.sleep();
    ctrl_output = test_jpc->compute_output( values[i], 0.0 );
    if(with_friction_compensation)
    {
      ROS_INFO_STREAM("Expected value: "<< expected_values_with_fc[i] << " Computed value: " <<ctrl_output);
      EXPECT_EQ(ctrl_output, expected_values_with_fc[i]);
    }
    else
    {
      ROS_INFO_STREAM("Expected value: "<< expected_values_no_fc[i] << " Computed value: " <<ctrl_output);
      EXPECT_EQ(ctrl_output, expected_values_no_fc[i]);
    }
  }

  //Test the position deadband as well:
  double target = 0.1;
  const double pos[5] = {0.0, 0.11, 0.099, 0.116, 0.115};
  const double expected_values[5] = {0.1, 0.0, 0.0, -0.016, 0.0};
  for(unsigned int i = 0; i < 5; ++i)
  {
    pause.sleep();
    ctrl_output = test_jpc->compute_output( target, pos[i] );
    ROS_INFO_STREAM("Expected value: "<< expected_values[i] << " Computed value: " <<ctrl_output);
    EXPECT_EQ(ctrl_output, expected_values[i]);
  }

  //TESTING A PURE I CONTROLLER
  pid->reset();
  pid->setGains(0.0, 1.0, 0.0, 2.0, -2.0);

  ros::Duration one_sec_pause(1.0);
  ros::Duration half_sec_pause(0.5);
  //ros::Duration pause(0.01);

  const double expected_values_one_sec[nb_values] = {-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0};
  const double expected_values_half_sec[nb_values] = {-2.0, -0.5, -0.25, 0.0, 0.25, 0.5, 2.0};

  for(unsigned int i = 0; i < nb_values; ++i)
  {
    ROS_INFO_STREAM("Sending demand: "<<values[i]);
    pid->reset();
    pause.sleep();
    ctrl_output = test_jpc->compute_output( 0.0, 0.0 );
    one_sec_pause.sleep();
    ctrl_output = test_jpc->compute_output( values[i], 0.0 );

    ROS_INFO_STREAM("Expected value: "<< expected_values_one_sec[i] << " Computed value: " <<ctrl_output);
    EXPECT_TRUE( fabs(ctrl_output - expected_values_one_sec[i]) < 0.001 );
  }
  for(unsigned int i = 0; i < nb_values; ++i)
  {
    ROS_INFO_STREAM("Sending demand: "<<values[i]);
    pid->reset();
    pause.sleep();
    ctrl_output = test_jpc->compute_output( 0.0, 0.0 );
    half_sec_pause.sleep();
    ctrl_output = test_jpc->compute_output( values[i], 0.0 );

    ROS_INFO_STREAM("Expected value: "<< expected_values_half_sec[i] << " Computed value: " <<ctrl_output);
    EXPECT_TRUE( fabs(ctrl_output - expected_values_half_sec[i]) < 0.001 );
  }
}


/////////////////////
//     MAIN       //
///////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_joint_pos_controller");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
