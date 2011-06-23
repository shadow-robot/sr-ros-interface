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

#include "sr_robot_lib/motor_updater.hpp"
#include <boost/foreach.hpp>
#include <iostream>

namespace motor_updater
{
  MotorUpdater::MotorUpdater(std::vector<UpdateConfig> update_configs_vector)
    : nh_tilde("~"), even_motors(1),
      which_data_from_motors(0)
  {
    mutex = boost::shared_ptr<boost::mutex>(new boost::mutex());

    BOOST_FOREACH(UpdateConfig config, update_configs_vector)
    {
      if(config.when_to_update != -1.0)
      {
        double tmp_dur = config.when_to_update;
        ros::Duration duration(tmp_dur);
        timers.push_back(nh_tilde.createTimer(duration, boost::bind(&MotorUpdater::timer_callback,
                                                                    this, _1, config.what_to_update)));
      }
      else
        important_update_configs_vector.push_back(config);
    }
  }

  void MotorUpdater::timer_callback(const ros::TimerEvent& event, FROM_MOTOR_DATA_TYPE data_type)
  {
    unimportant_data_queue.push(data_type);

    ROS_DEBUG_STREAM("Timer: data type = "<<data_type << " | queue size: "<<unimportant_data_queue.size());
  }


  MotorUpdater::~MotorUpdater()
  {

  }

  void MotorUpdater::build_update_motor_command(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command)
  {
    if(!mutex->try_lock())
      return;

    ///////
    // First we ask for the next data we want to receive
    if(even_motors)
      even_motors = 0;
    else
    {
      even_motors = 1;
      which_data_from_motors ++;

      if( which_data_from_motors >= important_update_configs_vector.size() )
        which_data_from_motors = 0;
    }

    command->which_motors = even_motors;

    if(!unimportant_data_queue.empty())
    {
      //an unimportant data is available
      command->from_motor_data_type = unimportant_data_queue.front();
      unimportant_data_queue.pop();

      ROS_DEBUG_STREAM("Updating unimportant data type: "<<command->from_motor_data_type << " | queue size: "<<unimportant_data_queue.size());
    }
    else
    {
      //important data to update as often as possible
      command->from_motor_data_type = important_update_configs_vector[which_data_from_motors].what_to_update;
      ROS_DEBUG_STREAM("Updating important data type: "<<command->from_motor_data_type << " | ["<<which_data_from_motors<<"/"<<important_update_configs_vector.size()<<"] ");
    }

    mutex->unlock();

  }
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
