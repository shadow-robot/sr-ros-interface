/**
 * @file   sr_real_tactile_sensor.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 10 11:07:10 2011
 * 
 * @brief  This is the virtual implementation of the SrGenericTactileSensor. It
 * computes virtual data.
 * 
 * 
 */


#ifndef _SR_VIRTUAL_TACTILE_SENSOR_HPP_
#define _SR_VIRTUAL_TACTILE_SENSOR_HPP_

#include <ros/ros.h>

#include "sr_tactile_sensors/sr_generic_tactile_sensor.hpp"

namespace shadowrobot
{
  class SrVirtualTactileSensor : public SrGenericTactileSensor
  {
  public:
    SrVirtualTactileSensor(std::string name, std::string touch_name, 
                           std::string temp_name);
    ~SrVirtualTactileSensor();

  protected:
    /**
     * Generates a value for the sensor
     *
     * @return the pressure value
     */
    virtual double get_touch_data();
    /**
     * Generates a value for the sensor
     *
     * @return the temperature value
     */
    virtual double get_temp_data();
  };

  class SrVirtualTactileSensorManager : public SrTactileSensorManager
  {
  public:
    SrVirtualTactileSensorManager();
    ~SrVirtualTactileSensorManager();
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
