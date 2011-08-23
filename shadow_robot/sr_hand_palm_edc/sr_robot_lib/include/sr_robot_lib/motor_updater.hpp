/**
 * @file   motor_updater.hpp
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

#ifndef _MOTOR_UPDATER_HPP_
#define _MOTOR_UPDATER_HPP_

#include <ros/ros.h>
#include <vector>
#include <list>
#include <queue>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include <sr_external_dependencies/types_for_external.h>
extern "C"
{
  #include <sr_external_dependencies/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h>
}

namespace motor_updater
{
  struct UpdateConfig
  {
    FROM_MOTOR_DATA_TYPE what_to_update;
    double when_to_update;
  };

  /**
   * The Motor Updater builds the next command we want to send to the hand.
   * We can ask for different types of data at different rates. The data and
   * their rates are defined in the sr_robot_lib/config/motor_data_polling.yaml
   * The important data are refreshed as often as possible (they have a -1. refresh
   * rate in the config file).
   *
   * The unimportant data are refreshed at their given rate (the value is defined in
   * the config in seconds).
   */
  class MotorUpdater
  {
  public:
    MotorUpdater(std::vector<UpdateConfig> update_configs_vector);
    ~MotorUpdater();

    /**
     * Building the motor command. This function is called at each packCommand() call.
     * If an unimportant data is waiting then we send it, otherwise, we send the next
     * important data.
     *
     * @param command The command which will be sent to the motor.
     */
    void build_update_motor_command(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command);

    /**
     * A timer callback for the unimportant data. The frequency of this callback
     * is defined in the config file.
     *
     * @param event
     * @param data_type The unimportant data type we want to ask for.
     */
    void timer_callback(const ros::TimerEvent& event, FROM_MOTOR_DATA_TYPE data_type);

  private:
    ros::NodeHandle nh_tilde;
    ///are we sending the command to the even or the uneven motors.
    int even_motors;

    ///Contains all the important data types.
    std::vector<UpdateConfig> important_update_configs_vector;
    ///iterate through the important data types.
    int which_data_from_motors;

    ///All the timers for the unimportant data types.
    std::vector<ros::Timer> timers;
    ///A queue containing the unimportant data types we want to ask for next time (empty most of the time).
    std::queue<FROM_MOTOR_DATA_TYPE, std::list<FROM_MOTOR_DATA_TYPE> > unimportant_data_queue;

    boost::shared_ptr<boost::mutex> mutex;
  };
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
