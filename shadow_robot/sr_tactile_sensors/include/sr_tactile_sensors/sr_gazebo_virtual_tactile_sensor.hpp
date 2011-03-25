/**
 * @file   sr_real_tactile_sensor.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com> updated by Ravin de Souza <rsouza@isr.ist.utl.pt>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 10 11:07:10 2011
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

#include <gazebo_plugins/ContactsState.h>

namespace shadowrobot
{
  class SrGazeboVirtualTactileSensor : public SrGenericTactileSensor
  {
  public:
    SrGazeboVirtualTactileSensor(std::string name, std::string gazebo_topic);
    ~SrGazeboVirtualTactileSensor();

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

  private:
    ros::NodeHandle nh;
    boost::mutex touch_mutex;
    double touch_value;

    ros::Subscriber sub;
    /** 
     * Callback function called when a msg is received on the
     * gazebo bumper topic.
     * 
     * @param msg the message containing the contact data
     */
    void callback(const gazebo_plugins::ContactsState& msg);
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
