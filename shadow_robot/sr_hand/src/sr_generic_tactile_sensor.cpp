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

  double SrGenericTactileSensor::get_temp_data()
  {
    return 0.0;
  }

  double SrGenericTactileSensor::get_touch_data()
  {
    return 0.0;
  }


/**********************************
 *     TACTILE SENSOR MANAGER     *
 **********************************/
  SrTactileSensorManager::SrTactileSensorManager() :
    n_tilde("~"), publish_rate(20.0)
  {
    init_tactile_sensors();

    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 20.0);
    publish_rate = ros::Rate(publish_freq);

  }

  void SrTactileSensorManager::init_tactile_sensors()
  {
    std::vector<std::string> names, sensor_touch_names, sensor_temp_names;
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

    for( unsigned int i=0; i<5; ++i)
    {
      tactile_sensors.push_back( SrGenericTactileSensor(names[i],
                                                        sensor_touch_names[i],
                                                        sensor_temp_names[i]) );
    }
  }

  SrTactileSensorManager::~SrTactileSensorManager()
  {}

  void SrTactileSensorManager::publish_all()
  {
    std::vector<SrGenericTactileSensor>::iterator tactile_sensor;

    for(tactile_sensor = tactile_sensors.begin() ;
        tactile_sensor != tactile_sensors.end() ; ++tactile_sensor)
      tactile_sensor->publish_current_values();

    publish_rate.sleep();
  }
}

/** 
 * The main function initializes the links with the robot, initializes
 * this ROS publisher regularly publishes data
 * regarding the finger tips tactile sensors
 * 
 * @param argc 
 * @param argv 
 * 
 * @return -1 if error linking with the robot (i.e. robot code not started)
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_tactile_sensor");
  ros::NodeHandle n;

  shadowrobot::SrTactileSensorManager tact_sens_mgr;

  while( ros::ok() )
    tact_sens_mgr.publish_all();

  return 0;
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
