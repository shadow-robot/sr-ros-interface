/**
 * @file   sr_virtual_tactile_sensor.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 10 11:07:10 2011
 *
 * @brief  This is the virtual implementation of the SrGenericTactileSensor. It
 * computes virtual data.
 *
 *
 */

#include "sr_tactile_sensors/sr_virtual_tactile_sensor.hpp"
#include <boost/algorithm/string.hpp>
#include <string>

namespace shadowrobot
{
/**********************************
 *         TACTILE SENSOR         *
 **********************************/
  SrVirtualTactileSensor::SrVirtualTactileSensor(std::string name,
                                                 std::string touch_name,
                                                 std::string temp_name) :
    SrGenericTactileSensor(name, touch_name, temp_name),
    touch_value(0.0), temp_value(0.0)
  {
    //fills the vector of joint names: we're taking J3 and J0
    std::string tmp = boost::to_upper_copy(name);
    tmp += "J3";
    names_joints_linked.push_back(tmp);
    tmp = boost::to_upper_copy(name);
    tmp += "J0";
    names_joints_linked.push_back(tmp);

    sub = nh.subscribe("/srh/shadowhand_data", 2, &SrVirtualTactileSensor::callback, this);
  }

  SrVirtualTactileSensor::~SrVirtualTactileSensor()
  {}

  void SrVirtualTactileSensor::callback(const sr_robot_msgs::joints_dataConstPtr& msg)
  {
    double tmp_value = 0.0;
    int msg_length = msg->joints_list_length;
    for(unsigned short index_msg=0; index_msg < msg_length; ++index_msg)
    {
      //get the sensor name
      std::string sensor_name = msg->joints_list[index_msg].joint_name;

      for(unsigned short index_name=0; index_name < names_joints_linked.size() ; ++index_name )
      {
        if(sensor_name.compare(names_joints_linked[index_name]) == 0)
        {
          //pressure value = sum(positions)
          tmp_value += msg->joints_list[index_msg].joint_position;
          break;
        }
      }
    }

    touch_mutex.lock();
    touch_value = tmp_value;
    touch_mutex.unlock();
  }

  double SrVirtualTactileSensor::get_touch_data()
  {
    double return_value;
    touch_mutex.lock();
    return_value = touch_value;
    touch_mutex.unlock();

    return return_value;
  }

  double SrVirtualTactileSensor::get_temp_data()
  {
    double return_value;
    temp_mutex.lock();
    return_value = temp_value;
    temp_mutex.unlock();

    return return_value;
  }


/**********************************
 *     TACTILE SENSOR MANAGER     *
 **********************************/
  SrVirtualTactileSensorManager::SrVirtualTactileSensorManager() :
    SrTactileSensorManager()
  {
    std::vector<std::vector<std::string> > all_names = get_all_names();

    for( unsigned int i=0; i<5; ++i)
    {
      tactile_sensors.push_back(
        boost::shared_ptr<SrVirtualTactileSensor>(
          new SrVirtualTactileSensor(all_names[0][i],
                                     all_names[1][i],
                                     all_names[2][i]) ));
    }
  }

  SrVirtualTactileSensorManager::~SrVirtualTactileSensorManager()
  {}
}
/**
 * Initializes a set of virtual tactile sensors and publish.
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

  shadowrobot::SrVirtualTactileSensorManager tact_sens_mgr;

  while( ros::ok() )
    tact_sens_mgr.publish_all();

  return 0;
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
