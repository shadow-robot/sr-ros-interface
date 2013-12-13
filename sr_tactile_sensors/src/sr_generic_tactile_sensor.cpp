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
    using namespace XmlRpc;
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 20.0);
    publish_rate = ros::Rate(publish_freq);

    //initializing the thresholds to test if the hand is holding
    //something or not (compared agains the pressure value).
    double tmp[5]={117,117,113,111,0};
    XmlRpc::XmlRpcValue threshold_xmlrpc;
    if(n_tilde.getParam("/grasp_touch_thresholds",threshold_xmlrpc))
    {
      ROS_ASSERT(threshold_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray);
      if(threshold_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if(threshold_xmlrpc.size() == 5)
        {
          for (int i = 0; i < threshold_xmlrpc.size(); ++i)
          {
            if(threshold_xmlrpc[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
              tmp[i] = static_cast<double>(threshold_xmlrpc[i]);
            else
              ROS_ERROR("grasp_touch_thresholds value %d is not a double, not loading",i+1);
          }
        }
        else
          ROS_ERROR("grasp_touch_thresholds must be of size 5, using default values");
      }
      else
        ROS_ERROR("grasp_touch_thresholds must be an array, using default values");
    }
    else
      ROS_WARN("grasp_touch_thresholds not set, using default values");

    is_hand_occupied_thresholds = std::vector<double>(tmp, tmp+5);
    ROS_DEBUG("is_hand_occupied_thresholds:[%f %f %f %f %f]", is_hand_occupied_thresholds.at(0),is_hand_occupied_thresholds.at(1),is_hand_occupied_thresholds.at(2),is_hand_occupied_thresholds.at(3),is_hand_occupied_thresholds.at(4));


    is_hand_occupied_server = n_tilde.advertiseService("is_hand_occupied", &SrTactileSensorManager::is_hand_occupied_cb, this);
    which_fingers_are_touching_server = n_tilde.advertiseService("which_fingers_are_touching", &SrTactileSensorManager::which_fingers_are_touching_cb, this);
  }

  SrTactileSensorManager::~SrTactileSensorManager()
  {}

  bool SrTactileSensorManager::is_hand_occupied_cb(sr_robot_msgs::is_hand_occupied::Request  &req,
                                                   sr_robot_msgs::is_hand_occupied::Response &res )
  {
    bool is_occupied = true;

    for(unsigned int i=0; i < tactile_sensors.size(); ++i)
    {
      if(tactile_sensors[i]->get_touch_data() < is_hand_occupied_thresholds[i])
      {
        is_occupied = false;
        ROS_DEBUG("is_hand_occupied_thresholds %d with val %f is smaller than threshold %f",i,tactile_sensors[i]->get_touch_data(),is_hand_occupied_thresholds[i]);
        break;
      }
    }

    res.hand_occupied = is_occupied;

    return true;
  }

  bool SrTactileSensorManager::which_fingers_are_touching_cb(sr_robot_msgs::which_fingers_are_touching::Request  &req,
                                                             sr_robot_msgs::which_fingers_are_touching::Response &res )
  {
    std::vector<double> touch_values(5);
    ROS_ASSERT(tactile_sensors.size() == 5);

    double value_tmp = 0.0;
    for(unsigned int i=0; i < tactile_sensors.size(); ++i)
    {
      value_tmp = tactile_sensors[i]->get_touch_data();
      if(value_tmp < req.force_thresholds[i])
        touch_values[i] = 0.0;
      else
        touch_values[i] = value_tmp;
    }
    res.touch_forces = touch_values;
    return true;
  }

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
