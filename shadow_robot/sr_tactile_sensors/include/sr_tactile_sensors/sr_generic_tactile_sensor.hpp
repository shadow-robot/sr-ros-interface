/**
 * @file   sr_generic_tactile_sensor.hpp
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
 * @brief  This is a generic parent class for the tactile sensors used in the
 * Shadow Robot Dextrous Hand. It implements virtual tactile sensors and is
 * extended in sr_real_tactile_sensor.hpp to implement the connection to real
 * sensors.
 * 
 * 
 */

#ifndef _SR_GENERIC_TACTILE_SENSOR_HPP_
#define _SR_GENERIC_TACTILE_SENSOR_HPP_

#include <ros/ros.h>

#include <boost/smart_ptr.hpp>
#include <std_msgs/Float64.h>

namespace shadowrobot
{
  class SrGenericTactileSensor
  {
  public:
    SrGenericTactileSensor(std::string name, std::string touch_name, 
                           std::string temp_name);
    ~SrGenericTactileSensor();

    /** 
     * publish the current values to the
     * correct ros topics
     */
    void publish_current_values();

  protected:
    /** 
     * Needs to be implemented in the inheriting class
     * 
     * @return the pressure value
     */
    virtual double get_touch_data() = 0;
    /** 
     * Needs to be implemented in the inheriting class
     * 
     * @return the temperature value
     */
    virtual double get_temp_data() = 0;

  private:
    ros::Publisher touch_pub, temp_pub;
    ros::NodeHandle n_tilde;
    std::string touch_sensor_name, temp_sensor_name;
    std_msgs::Float64 msg_temp, msg_touch;
  };

  class SrTactileSensorManager
  {
  public:
    SrTactileSensorManager();
    ~SrTactileSensorManager();

    /** 
     * Calls the publish_current_values for each of the tactile
     * sensors.
     */
    void publish_all();

  protected:
    std::vector<boost::shared_ptr<SrGenericTactileSensor> > tactile_sensors;
    ros::NodeHandle n_tilde;
    ros::Rate publish_rate;

    /** 
     * Get all the necessary names for the tactile sensors:
     * the display names, the touch sensor name (for the robot),
     * the temperature sensor name (for the robot).
     *
     * Those names are read from the parameter server. They are stored
     * in params/sensor_names.yaml
     * 
     * @return a vector containing 3 vectors for the display_names,
     *         the sensor_touch_names and the sensor_temp_names ; in
     *         this order.
     */
    std::vector<std::vector<std::string> > get_all_names();
  };
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
