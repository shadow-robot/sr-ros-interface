/**
 * @file   sr_self_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Feb 4, 2013
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
 * @brief Class containing the self tests for the Shadow Robot EtherCAT hardware.
 *
 *
 */

#include "sr_self_test/sr_self_test.hpp"

namespace shadow_robot
{
  SrSelfTest::SrSelfTest(bool simulated)
  {
    test_runner_.setID("12345");

    std::vector<std::string> services_to_test;
    services_to_test.push_back("/pr2_controller_manager/list_controller_types");
    services_to_test.push_back("/pr2_controller_manager/list_controllers");
    services_to_test.push_back("/pr2_controller_manager/load_controller");
    services_to_test.push_back("/pr2_controller_manager/reload_controller_libraries");
    services_to_test.push_back("/pr2_controller_manager/switch_controller");
    services_to_test.push_back("/pr2_controller_manager/unload_controller");

    if( simulated )
    {
      services_to_test.push_back("/sh_ffj0_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_ffj0_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_ffj3_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_ffj3_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_ffj4_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_ffj4_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_lfj0_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_lfj0_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_lfj3_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_lfj3_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_lfj4_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_lfj4_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_lfj5_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_lfj5_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_mfj0_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_mfj0_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_mfj3_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_mfj3_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_mfj4_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_mfj4_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_rfj0_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_rfj0_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_rfj3_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_rfj3_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_rfj4_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_rfj4_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_thj1_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_thj1_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_thj2_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_thj2_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_thj3_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_thj3_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_thj4_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_thj4_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_thj5_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_thj5_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_wrj1_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_wrj1_mixed_position_velocity_controller/set_gains");
      services_to_test.push_back("/sh_wrj2_mixed_position_velocity_controller/reset_gains");
      services_to_test.push_back("/sh_wrj2_mixed_position_velocity_controller/set_gains");
    }

    test_runner_.addServicesTest(services_to_test);
  }

  void SrSelfTest::check_movements()
  {

  }
}  // namespace shadow_robot


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


