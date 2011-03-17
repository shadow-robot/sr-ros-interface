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

#include "sr_tactile_sensors/sr_generic_tactile_sensor.hpp"

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

    //initializing the thresholds to test if the hand is holding
    //something or not (compared agains the pressure value).
    double tmp[5]={50,50,50,50,0};
    is_hand_occupied_thresholds = std::vector<double>(tmp, tmp+5);

    is_hand_occupied_server = n_tilde.advertiseService("is_hand_occupied", &SrTactileSensorManager::is_hand_occupied_cb, this);
  }

  SrTactileSensorManager::~SrTactileSensorManager()
  {}

  bool SrTactileSensorManager::is_hand_occupied_cb(sr_robot_msgs::is_hand_occupied::Request  &req,
                                                   sr_robot_msgs::is_hand_occupied::Response &res )
  {
    bool is_occupied = true;

    for(unsigned int i=0; i < tactile_sensors.size(); ++i)
    {
      ROS_ERROR("%f, %f", tactile_sensors[i]->get_touch_data(), is_hand_occupied_thresholds[i] );

      if(tactile_sensors[i]->get_touch_data() < is_hand_occupied_thresholds[i])
      {
        is_occupied = false;
        break;
      }
    }

    res.hand_occupied = is_occupied;

    return true;
  }

  std::vector<std::vector<std::string> >  SrTactileSensorManager::get_all_names()
  {
    std::vector<std::vector<std::string> > all_names_vector;
    std::vector<std::string> names, sensor_touch_names, sensor_temp_names;

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

    n_tilde.getParam("sensor_temp_names", my_list);
    if(my_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
      bad_params = true;
    if(my_list.size() != list_size)
      bad_params = true;
    for (int32_t i = 0; i < list_size; ++i)
    {
      if(my_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
        bad_params = true;
      sensor_temp_names.push_back( static_cast<std::string>(my_list[i]) );
    }

    if(bad_params)
    {
      ROS_ERROR("Error while reading the parameter for the tactile sensors; using standard parameters");
      names.clear();
      sensor_touch_names.clear();
      sensor_temp_names.clear();

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

      sensor_temp_names.push_back("FF_Touch_Temp");
      sensor_temp_names.push_back("MF_Touch_Temp");
      sensor_temp_names.push_back("RF_Touch_Temp");
      sensor_temp_names.push_back("LF_Touch_Temp");
      sensor_temp_names.push_back("TH_Touch_Temp");
    }

    all_names_vector.push_back(names);
    all_names_vector.push_back(sensor_touch_names);
    all_names_vector.push_back(sensor_temp_names);

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
