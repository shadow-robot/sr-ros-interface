/**
 * @file   sr_real_tactile_sensor.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 10 11:07:10 2011
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
 * @brief  This is the implementation of the SrGenericTactileSensor. It
 * fetches data from the real sensors.
 *
 *
 */


#ifndef _SR_REAL_TACTILE_SENSOR_HPP_
#define _SR_REAL_TACTILE_SENSOR_HPP_

#include <ros/ros.h>

//our robot code
#include <robot/robot.h>
#include <robot/hand.h>
#include <robot/hand_protocol.h>

#include "sr_tactile_sensors/sr_generic_tactile_sensor.hpp"

namespace shadowrobot
{
  class SrRealTactileSensor : public SrGenericTactileSensor
  {
  public:
    SrRealTactileSensor(std::string name, std::string touch_name);
    virtual ~SrRealTactileSensor();

    /**
     * Reads the value from the sensor
     *
     * @return the pressure value ; -1000 if sensor not found
     */
    virtual double get_touch_data();

  private:
    struct sensor sensor_touch;
    int res_touch;
  };

  class SrRealTactileSensorManager : public SrTactileSensorManager
  {
  public:
    SrRealTactileSensorManager();
    ~SrRealTactileSensorManager();
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
