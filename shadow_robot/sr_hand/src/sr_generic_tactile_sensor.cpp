/**
 * @file   sr_generic_tactile_sensor.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 10 11:07:10 2011
 * 
 * @brief  This is a generic parent class for the tactile sensors used in the
 * Shadow Robot Dextrous Hand. 
 * 
 * 
 */

#include "sr_hand/sr_generic_tactile_sensor.hpp"

namespace shadowrobot
{
/**********************************
 *         TACTILE SENSOR         *
 **********************************/
  SrGenericTactileSensor::SrGenericTactileSensor(std::string name,
                                                 std::string touch_name,
                                                 std::string temp_name) :
    n_tilde("~")
  {
    std::string full_topic_touch = "touch/"+name;
    std::string full_topic_temp = "temp/"+name;

    touch_pub = n_tilde.advertise<std_msgs::Float64>(full_topic_touch, 10);
    temp_pub = n_tilde.advertise<std_msgs::Float64>(full_topic_temp, 10);
  }

  SrGenericTactileSensor::~SrGenericTactileSensor()
  {}

  void SrGenericTactileSensor::publish_current_values()
  {
    msg_temp.data = get_temp_data();
    msg_touch.data = get_touch_data();

    touch_pub.publish(msg_touch);
    temp_pub.publish(msg_temp);
  }

/**********************************
 *     TACTILE SENSOR MANAGER     *
 **********************************/
  SrTactileSensorManager::SrTactileSensorManager() :
    n_tilde("~"), publish_rate(20.0)
  {
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 20.0);
    publish_rate = ros::Rate(publish_freq);
  }

  SrTactileSensorManager::~SrTactileSensorManager()
  {}

  void SrTactileSensorManager::publish_all()
  {
    for(unsigned int i=0; i < tactile_sensors.size(); ++i)
      tactile_sensors[i].publish_current_values();

    publish_rate.sleep();
    ros::spinOnce();
  }
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
