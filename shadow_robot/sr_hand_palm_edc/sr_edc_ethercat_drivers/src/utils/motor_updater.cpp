/**
 * @file   motor_updater.cpp
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
 * @brief  This contains a class used to determin which data we should ask the motor for,
 * depending on the config we're using.
 *
 *
 */

#include "sr_edc_ethercat_drivers/utils/motor_updater.hpp"
#include <boost/foreach.hpp>
#include <iostream>

namespace motor_updater
{
  MotorUpdater::MotorUpdater()
    : even_motors(true), counter(0)
  {
    UpdateConfig test;
    test.what_to_update = 0;
    test.when_to_update = 1;
    test.is_important = true;

    update_configs_vector.push_back(test);

/*
  XmlRpc::XmlRpcValue my_list;
  nodehandle_.getParam("less_important_frequencies", my_list);
  ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int32_t i = 0; i < my_list.size(); ++i)
  {
  ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
  ROS_ERROR_STREAM("TOTO: " << static_cast<int>(my_list[i]));
  }
*/

  }

  MotorUpdater::~MotorUpdater()
  {

  }

  void MotorUpdater::build_update_motor_command(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command)
  {
    BOOST_FOREACH(UpdateConfig config, update_configs_vector)
    {
      if(config.is_important)
      {
        std::cout <<"Toto" <<std::endl;
      }
    }
  }
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
