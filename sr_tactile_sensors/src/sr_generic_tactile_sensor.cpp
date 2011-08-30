/**
 * @file   sr_generic_tactile_sensor.cpp
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
 * Shadow Robot Dextrous Hand.
 *
 *
 */

#include "sr_tactile_sensors/sr_generic_tactile_sensor.hpp"

namespace shadowrobot
{
/**********************************
 *         TACTILE SENSOR         *
 **********************************/
  SrGenericTactileSensor::SrGenericTactileSensor(std::string name,
                                                 std::string touch_name ) :
    n_tilde("~")
  {
    std::string full_topic_touch = "touch/"+name;

    touch_pub = n_tilde.advertise<std_msgs::Float64>(full_topic_touch, 10);
  }

  SrGenericTactileSensor::~SrGenericTactileSensor()
  {}

  void SrGenericTactileSensor::publish_current_values()
  {
    msg_touch.data = get_touch_data();

    touch_pub.publish(msg_touch);
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

  std::vector<std::vector<std::string> >  SrTactileSensorManager::get_all_names()
  {
    std::vector<std::vector<std::string> > all_names_vector;
    std::vector<std::string> names, sensor_touch_names;

    std::string tmp;
    XmlRpc::XmlRpcValue my_list;
    int list_size;
    bool bad_params = false;

    n_tilde.getParam("display_names", my_list);
    if(my_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
      bad_params = true;
    list_size = my_list.size();
    for (int32_t i = 0; i < list_size; ++i)
    {
      if(my_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        bad_params = true;
      names.push_back( static_cast<std::string>(my_list[i]) );
    }

    n_tilde.getParam("sensor_touch_names", my_list);
    if(my_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
      bad_params = true;
    if(my_list.size() != list_size)
      bad_params = true;
    list_size = my_list.size();
    for (int32_t i = 0; i < list_size; ++i)
    {
      if(my_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        bad_params = true;
      sensor_touch_names.push_back( static_cast<std::string>(my_list[i]) );
    }

    if(bad_params)
    {
      ROS_ERROR("Error while reading the parameter for the tactile sensors; using standard parameters");
      names.clear();
      sensor_touch_names.clear();

      names.push_back("ff");
      names.push_back("mf");
      names.push_back("rf");
      names.push_back("lf");
      names.push_back("th");

      sensor_touch_names.push_back("FF_Touch");
      sensor_touch_names.push_back("MF_Touch");
      sensor_touch_names.push_back("RF_Touch");
      sensor_touch_names.push_back("LF_Touch");
      sensor_touch_names.push_back("TH_Touch");
    }

    all_names_vector.push_back(names);
    all_names_vector.push_back(sensor_touch_names);

    return all_names_vector;
  }

  void SrTactileSensorManager::publish_all()
  {
    for(unsigned int i=0; i < tactile_sensors.size(); ++i)
      tactile_sensors[i]->publish_current_values();

    publish_rate.sleep();
    ros::spinOnce();
  }
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
