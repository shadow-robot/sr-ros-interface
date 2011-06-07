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

#include "sr_edc_ethercat_drivers/utils/motor_updater.hpp"
#include <gtest/gtest.h>

TEST(Utils, motor_updater_freq)
{
  motor_updater::MotorUpdater test = motor_updater::MotorUpdater();

  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command = new ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND();
  test.build_update_motor_command(command);

  EXPECT_EQ(0,0);
  delete command;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
