/**
 * @file   sr_edc_ethercat_drivers_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, <contact@shadowrobot.com>
 * @date   Tue Jun  7 09:15:21 2011
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
 * @brief  This is a set of unit tests for the ethercat drivers.
 *
 *
 */

#include "sr_robot_lib/motor_updater.hpp"
#include <gtest/gtest.h>

struct UpdaterResult
{
  bool svn_transmitted;
  bool svn_transmitted_once;
  int can_num_transmitted_counter;
};

class MotorUpdaterTest
{
public:
  MotorUpdaterTest()
  {}

  ~MotorUpdaterTest()
  {}

  UpdaterResult check_updates(double tolerancy)
  {
    std::vector<motor_updater::UpdateConfig> update_configs_vector;

    motor_updater::UpdateConfig test;
    test.what_to_update = MOTOR_DATA_SGL;
    test.when_to_update = -1.0;
    update_configs_vector.push_back(test);

    motor_updater::UpdateConfig test2;
    test2.what_to_update = MOTOR_DATA_SVN_REVISION;
    test2.when_to_update = 5.0;
    update_configs_vector.push_back(test2);

    motor_updater::UpdateConfig test3;
    test3.what_to_update = MOTOR_DATA_CAN_NUM_RECEIVED;
    test3.when_to_update = 1.0;
    update_configs_vector.push_back(test3);

    motor_updater::MotorUpdater motor_updater = motor_updater::MotorUpdater(update_configs_vector);

    ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command = new ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND();
    motor_updater.build_update_motor_command(command);

    bool svn_transmitted = false;
    bool svn_transmitted_once = false;

    int can_num_transmitted_counter = 0;

    ros::Time start = ros::Time::now();
    ros::Duration time_spent(0.0);

    while(time_spent.toSec() < 7.2)
    {
      ros::spinOnce();
      motor_updater.build_update_motor_command(command);

      time_spent = ros::Time::now() - start;

      if(abs(time_spent.toSec() - 5.0) < tolerancy )
      {
        if(command->from_motor_data_type == MOTOR_DATA_SVN_REVISION)
        {
          ROS_INFO_STREAM("Correct data received at time : "<<time_spent);
          svn_transmitted = true;

          if(svn_transmitted_once)
            svn_transmitted_once = false;
          else
            svn_transmitted_once = true;
        }
      }


      if(abs(time_spent.toSec()-((double)( (int)time_spent.toSec() ) ) ) < tolerancy )
      {
        if(command->from_motor_data_type == MOTOR_DATA_CAN_NUM_RECEIVED)
        {
          ROS_INFO_STREAM("Correct CAN data received at time : "<<time_spent);
          can_num_transmitted_counter ++;
        }
      }
      usleep(1000);
    }

    UpdaterResult updater_result;
    updater_result.svn_transmitted = svn_transmitted;
    updater_result.svn_transmitted_once = svn_transmitted_once;
    updater_result.can_num_transmitted_counter = can_num_transmitted_counter;

    delete command;

    return updater_result;
  }
};

TEST(Utils, motor_updater_freq_low_tolerancy)
{
  MotorUpdaterTest mut = MotorUpdaterTest();
  UpdaterResult updater_result = mut.check_updates(0.01);

  EXPECT_TRUE(updater_result.svn_transmitted);
  EXPECT_TRUE(updater_result.svn_transmitted_once);

  EXPECT_EQ(updater_result.can_num_transmitted_counter, 7);
}

TEST(Utils, motor_updater_freq_medium_tolerancy)
{
  MotorUpdaterTest mut = MotorUpdaterTest();
  UpdaterResult updater_result = mut.check_updates(0.05);

  EXPECT_TRUE(updater_result.svn_transmitted);
  EXPECT_TRUE(updater_result.svn_transmitted_once);

  EXPECT_EQ(updater_result.can_num_transmitted_counter, 7);
}

TEST(Utils, motor_updater_freq_high_tolerancy)
{
  MotorUpdaterTest mut = MotorUpdaterTest();
  UpdaterResult updater_result = mut.check_updates(0.1);

  EXPECT_TRUE(updater_result.svn_transmitted);
  EXPECT_TRUE(updater_result.svn_transmitted_once);

  EXPECT_EQ(updater_result.can_num_transmitted_counter, 7);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr_edc_ethercat_drivers_test");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
