/**
 * @file   sr_real_tactile_sensor.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 10 11:07:10 2011
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

#include "sr_hand/sr_generic_tactile_sensor.hpp"

namespace shadowrobot
{
  class SrRealTactileSensor : public SrGenericTactileSensor
  {
  public:
    SrRealTactileSensor(std::string name, std::string touch_name, 
                        std::string temp_name);
    ~SrRealTactileSensor();

  protected:
    /**
     * Reads the value from the sensor
     *
     * @return the pressure value
     */
    virtual double get_touch_data();
    /**
     * Reads the value from the sensor
     *
     * @return the pressure value
     */
    virtual double get_temp_data();

  private:
    struct sensor sensor_touch, sensor_temp;
    int res_touch, res_temp;
  };

  class SrRealTactileSensorManager : public SrTactileSensorManager
  {
  public:
    SrRealTactileSensorManager();
    ~SrRealTactileSensorManager();

  protected:
    virtual void init_tactile_sensors();
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
