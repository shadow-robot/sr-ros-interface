/**
 * @file   test_robot_lib.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Jun 22 13:04:41 2011
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
 * @brief This is a set of unit tests testing the robot libraries.
 *
 *
 */

#include "sr_robot_lib/sr_hand_lib.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

class HandLibTest
{
public:
  pr2_hardware_interface::HardwareInterface *hw;
  boost::shared_ptr<shadow_robot::SrHandLib> sr_hand_lib;
  pr2_hardware_interface::Actuator* actuator;

  HandLibTest()
  {
    hw = new pr2_hardware_interface::HardwareInterface();
    sr_hand_lib= boost::shared_ptr<shadow_robot::SrHandLib>( new shadow_robot::SrHandLib(hw) );
  }

  ~HandLibTest()
  {
    delete hw;
  }

  void check_hw_actuator(std::string name, int motor_id, int id_in_enum, double expected_pos)
  {
    pr2_hardware_interface::ActuatorState state;

    actuator = hw->getActuator(name);
    state = (actuator->state_);

    EXPECT_EQ(state.device_id_ , motor_id);
    EXPECT_EQ(state.last_measured_effort_ , (double)motor_id / 2.0);
    EXPECT_EQ(state.position_ , expected_pos);
    EXPECT_EQ(state.motor_voltage_, 10.0*(double)id_in_enum/256.0);
  }

};

/**
 * Tests the initialization of the hand library.
 */
TEST(SrRobotLib, Initialization)
{
  boost::shared_ptr< HandLibTest > lib_test = boost::shared_ptr< HandLibTest >( new HandLibTest() );

  EXPECT_EQ(lib_test->sr_hand_lib->joints_vector.size(), 28);
}

/**
 * Tests the update of the hand library.
 */
TEST(SrRobotLib, UpdateMotor)
{
  boost::shared_ptr< HandLibTest > lib_test = boost::shared_ptr< HandLibTest >( new HandLibTest() );

  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data = new ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS();
  //add growing sensors values
  for(unsigned int i=1 ; i < SENSORS_NUM_0220 + 2; ++i)
  {
    //position = id in joint enum
    status_data->sensors[i] = i;
  }

  status_data->motor_data_type = MOTOR_DATA_SGR;

  //even motors
  status_data->which_motors = 0;

  //all motor data arrived with no errors
  status_data->which_motor_data_arrived = 0x00055555;
  status_data->which_motor_data_had_errors = 0;

  //add growing motor data packet values
  for(unsigned int i=0 ; i < 10; ++i)
  {
    status_data->motor_data_packet[i].torque = 4;
    status_data->motor_data_packet[i].misc = 2*i;
  }
  //filling the status data with known values
  status_data->idle_time_us = 1;

  //update the library
  lib_test->sr_hand_lib->update(status_data);

  //check the data we read back are correct.
  EXPECT_EQ(lib_test->sr_hand_lib->main_pic_idle_time, 1);
  EXPECT_EQ(lib_test->sr_hand_lib->main_pic_idle_time_min, 1);

  //check the sensors etc..
  boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp = lib_test->sr_hand_lib->joints_vector.begin();
  for(;joint_tmp != lib_test->sr_hand_lib->joints_vector.end(); ++joint_tmp)
  {
    if(joint_tmp->has_motor)
    {
      //we updated the even motors
      if(joint_tmp->motor->motor_id % 2 == 0)
      {
        EXPECT_EQ(joint_tmp->motor->actuator->state_.last_measured_effort_ , 4.0);//(double)joint_tmp->motor->motor_id/2.0);
        EXPECT_EQ(joint_tmp->motor->strain_gauge_right, joint_tmp->motor->motor_id);
      }
    }
  }
}

/**
 * Tests the update of the actuators
 * which are in the pr2_hardware_interface hw*
 */
TEST(SrRobotLib, UpdateActuators)
{
  boost::shared_ptr< HandLibTest > lib_test = boost::shared_ptr< HandLibTest >( new HandLibTest() );

  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data = new ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS();
  //add growing sensors values
  for(unsigned int i=0 ; i < SENSORS_NUM_0220 + 1; ++i)
  {
    //position = id in joint enum
    status_data->sensors[i] = i+1;
  }

  status_data->motor_data_type = MOTOR_DATA_VOLTAGE;

  //even motors
  status_data->which_motors = 0;

  //all motor data arrived with no errors
  status_data->which_motor_data_arrived = 0x00055555;
  status_data->which_motor_data_had_errors = 0;

  //add growing motor data packet values
  for(unsigned int i=0 ; i < 10; ++i)
  {
    status_data->motor_data_packet[i].torque = i;
    status_data->motor_data_packet[i].misc = 10*i;
  }
  //filling the status data with known values
  status_data->idle_time_us = 1;

  //update the library
  lib_test->sr_hand_lib->update(status_data);

  lib_test->check_hw_actuator("FFJ4", 2, 1, 4.0);
  lib_test->check_hw_actuator("MFJ3", 4, 2, 7.0);

  //cleanup
  delete status_data;
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

