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
        EXPECT_EQ(joint_tmp->motor->actuator->state_.strain_gauge_right_, joint_tmp->motor->motor_id);
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

/**
 * For the next tests we want to have access to the calibrate_joint
 * method which is protected in our code.
 * The method is found at:
 * http://code.google.com/p/googletest/wiki/V1_6_FAQ#How_do_I_test_private_class_members_without_writing_FRIEND_TEST(
 */
class TestHandLib
  : public shadow_robot::SrHandLib
{
public:
  TestHandLib(pr2_hardware_interface::HardwareInterface* hw)
    : SrHandLib(hw)
  {}

  using shadow_robot::SrHandLib::calibrate_joint;

  using shadow_robot::SrHandLib::status_data;

  using shadow_robot::SrHandLib::actuator;

  using shadow_robot::SrHandLib::humanize_flags;
};


/**
 * Testing the calibration procedure for
 * a joint having one motor only (FFJ3)
 *
 */
TEST(SrRobotLib, CalibrationOneMotor)
{

  pr2_hardware_interface::HardwareInterface *hw;
  boost::shared_ptr<TestHandLib> sr_hand_lib = boost::shared_ptr<TestHandLib>( new TestHandLib(hw) );

  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS status_data;

  //set all the sensors to 0
  for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
    status_data.sensors[i] = 0;

  sr_hand_lib->status_data = &status_data;
  for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
    EXPECT_EQ(sr_hand_lib->status_data->sensors[i], 0);

  //let's find FFJ3 in the vector
  boost::ptr_vector<shadow_joints::Joint>::iterator ffj3 = sr_hand_lib->joints_vector.begin();
  std::string name_tmp = ffj3->joint_name;
  bool ffj3_found = false;
  int index_ffj3 = 0;

  for(; ffj3 != sr_hand_lib->joints_vector.end(); ++ffj3)
  {
    name_tmp = ffj3->joint_name;

    if( name_tmp.compare("FFJ3") == 0)
    {
      ffj3_found = true;
      break;
    }

    ++index_ffj3;
  }

  EXPECT_TRUE(ffj3_found);
  EXPECT_EQ(index_ffj3, 3);

  sr_hand_lib->actuator = (ffj3->motor->actuator);

  sr_hand_lib->calibrate_joint(ffj3);
  //all the sensors at 0 -> should be 0
  EXPECT_EQ( ffj3->motor->actuator->state_.position_ , 0.0);

  for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
  sr_hand_lib->status_data->sensors[i] = 1;

  //now ffj3 position should be 1
  sr_hand_lib->calibrate_joint(ffj3);
  //all the sensors at 1 -> should be 1
  EXPECT_EQ( ffj3->motor->actuator->state_.position_ , 1.0);

  delete hw;
}


/**
 * Testing the calibration procedure for
 * a joint having calibrating the sensors first
 * and then combining them (FFJ0)
 *
 */
TEST(SrRobotLib, CalibrationFFJ0)
{

  pr2_hardware_interface::HardwareInterface *hw;
  boost::shared_ptr<TestHandLib> sr_hand_lib = boost::shared_ptr<TestHandLib>( new TestHandLib(hw) );

  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS status_data;

  //set all the sensors to 0
  for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
    status_data.sensors[i] = 0;

  sr_hand_lib->status_data = &status_data;
  for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
    EXPECT_EQ(sr_hand_lib->status_data->sensors[i], 0);

  //let's find FFJ0 in the vector
  boost::ptr_vector<shadow_joints::Joint>::iterator ffj0 = sr_hand_lib->joints_vector.begin();
  std::string name_tmp = ffj0->joint_name;
  bool ffj0_found = false;
  int index_ffj0 = 0;

  for(; ffj0 != sr_hand_lib->joints_vector.end(); ++ffj0)
  {
    name_tmp = ffj0->joint_name;

    if( name_tmp.compare("FFJ0") == 0)
    {
      ffj0_found = true;
      break;
    }

    ++index_ffj0;
  }

  EXPECT_TRUE(ffj0_found);
  EXPECT_EQ(index_ffj0, 0);

  sr_hand_lib->actuator = (ffj0->motor->actuator);

  sr_hand_lib->calibrate_joint(ffj0);
  //all the sensors at 0 -> should be 0
  EXPECT_EQ( ffj0->motor->actuator->state_.position_ , 0.0);

  for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
    sr_hand_lib->status_data->sensors[i] = 1;

  //now ffj0 position should be 1
  sr_hand_lib->calibrate_joint(ffj0);
  //all the sensors at 1 -> should be 2
  EXPECT_EQ( ffj0->motor->actuator->state_.position_ , 2.0);

  delete hw;
}

/**
 * Testing the calibration procedure for
 * a compound joint combining the sensors
 * and then calibrating the total (THJ5)
 *
 */
TEST(SrRobotLib, CalibrationTHJ5)
{

  pr2_hardware_interface::HardwareInterface *hw;
  boost::shared_ptr<TestHandLib> sr_hand_lib = boost::shared_ptr<TestHandLib>( new TestHandLib(hw) );

  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS status_data;

  //set all the sensors to 0
  for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
    status_data.sensors[i] = 0;

  sr_hand_lib->status_data = &status_data;
  for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
    EXPECT_EQ(sr_hand_lib->status_data->sensors[i], 0);

  //let's find THJ5 in the vector
  boost::ptr_vector<shadow_joints::Joint>::iterator thj5 = sr_hand_lib->joints_vector.begin();
  std::string name_tmp = thj5->joint_name;
  bool thj5_found = false;
  int index_thj5 = 0;

  for(; thj5 != sr_hand_lib->joints_vector.end(); ++thj5)
  {
    name_tmp = thj5->joint_name;

    if( name_tmp.compare("THJ5") == 0)
    {
      thj5_found = true;
      break;
    }

    ++index_thj5;
  }

  EXPECT_TRUE(thj5_found);
  EXPECT_EQ(index_thj5, 25);

  sr_hand_lib->actuator = (thj5->motor->actuator);

  sr_hand_lib->calibrate_joint(thj5);
  //all the sensors at 0 -> should be 0
  EXPECT_EQ( thj5->motor->actuator->state_.position_ , 0.0);

  for(unsigned int i = 0; i < SENSORS_NUM_0220 + 1; ++i)
    sr_hand_lib->status_data->sensors[i] = 1;

  //now thj5 position should be 1
  sr_hand_lib->calibrate_joint(thj5);
  //all the sensors at 1 -> should be 1 (THJ5 = .5 THJ5A + .5 THJ5B)
  EXPECT_EQ( thj5->motor->actuator->state_.position_ , 1.0);

  delete hw;
}

/**
 * Testing the humanization of the flags.
 *
 */
TEST(SrRobotLib, HumanizeFlags)
{
  pr2_hardware_interface::HardwareInterface *hw;
  boost::shared_ptr<TestHandLib> sr_hand_lib = boost::shared_ptr<TestHandLib>( new TestHandLib(hw) );

  std::vector<std::pair<std::string, bool> > flags;
  //all flags set
  flags = sr_hand_lib->humanize_flags(0xFFFF);

  EXPECT_EQ(flags.size(), 16);

  for(unsigned int i=0; i < 16; ++i)
    EXPECT_EQ(flags[i].first.compare(error_flag_names[i]) , 0);

  //The last three flags are serious
  EXPECT_TRUE(flags[13].second);
  EXPECT_TRUE(flags[14].second);
  EXPECT_TRUE(flags[15].second);
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

