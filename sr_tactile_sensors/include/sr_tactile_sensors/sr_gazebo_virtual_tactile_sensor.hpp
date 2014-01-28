/**
 * @file   sr_real_tactile_sensor.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com> updated by Ravin de Souza <rsouza@isr.ist.utl.pt>, Contact <contact@shadowrobot.com>
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
 * @brief  This is the virtual implementation of the SrGenericTactileSensor. It
 * computes virtual data.
 *
 *
 */


#ifndef _SR_GAZEBO_VIRTUAL_TACTILE_SENSOR_HPP_
#define _SR_GAZEBO_VIRTUAL_TACTILE_SENSOR_HPP_

#include <ros/ros.h>

#include <boost/thread.hpp>

#include "sr_tactile_sensors/sr_generic_tactile_sensor.hpp"

#include <gazebo_msgs/ContactsState.h>

namespace shadowrobot
{
  class SrGazeboVirtualTactileSensor : public SrGenericTactileSensor
  {
  public:
    SrGazeboVirtualTactileSensor(std::string name, std::string gazebo_topic);
    virtual ~SrGazeboVirtualTactileSensor();

  protected:
    /**
     * Generates a value for the sensor
     *
     * @return the pressure value
     */
    virtual double get_touch_data();

  private:
    ros::NodeHandle nh;
    boost::mutex touch_mutex;
    double touch_value;
    bool touch_freshdata;

    ros::Subscriber sub;
    /**
     * Callback function called when a msg is received on the
     * gazebo bumper topic.
     *
     * @param msg the message containing the contact data
     */
    void callback(const gazebo_msgs::ContactsState& msg);
  };

  class SrGazeboVirtualTactileSensorManager : public SrTactileSensorManager
  {
  public:
    SrGazeboVirtualTactileSensorManager();
    ~SrGazeboVirtualTactileSensorManager();
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
