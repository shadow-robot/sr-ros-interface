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
 * @brief  This is the virtual implementation of the SrGenericTactileSensor. It
 * computes virtual data.
 *
 *
 */


#ifndef _SR_VIRTUAL_TACTILE_SENSOR_HPP_
#define _SR_VIRTUAL_TACTILE_SENSOR_HPP_

#include <ros/ros.h>

#include <boost/thread.hpp>

#include "sr_tactile_sensors/sr_generic_tactile_sensor.hpp"

#include <sr_robot_msgs/joints_data.h>
#include <sr_robot_msgs/joint.h>

namespace shadowrobot
{
  class SrVirtualTactileSensor : public SrGenericTactileSensor
  {
  public:
    /**
     * The Virtual Tactile sensors. For those sensor, we subscribe to
     * the data coming from the hand and update the tactile sensor
     * value based on this information. This should be extended to subscribe
     * to a Gazebo touch sensor.
     *
     * @param name the display name of the sensor
     * @param touch_name the actual name of the touch sensor
     *
     * @return
     */
    SrVirtualTactileSensor(std::string name, std::string touch_name);
    ~SrVirtualTactileSensor();

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

    /**
     * The names from which we get the joint position.
     */
    std::vector<std::string> names_joints_linked;
    /**
     * Subscribes to the shadowhand_data topic and updates
     * the touch_value based on the joint positions
     */
    ros::Subscriber sub;
    /**
     * Callback function called when a msg is received on the
     * shadowhand__data topic. Update the touch_value based on
     * the joint positions of the joints contained in
     * names_joints_linked.
     *
     * @param msg the message containing the joint positions
     */
    void callback(const sr_robot_msgs::joints_dataConstPtr& msg);
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
