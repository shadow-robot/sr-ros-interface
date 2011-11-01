/**
 * @file   sr_virtual_tactile_sensor.cpp
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

#include "sr_tactile_sensors/sr_virtual_tactile_sensor.hpp"
#include <boost/algorithm/string.hpp>
#include <string>

namespace shadowrobot
{
/**********************************
 *         TACTILE SENSOR         *
 **********************************/
  SrVirtualTactileSensor::SrVirtualTactileSensor(std::string name,
                                                 std::string touch_name ) :
    SrGenericTactileSensor(name, touch_name),
    touch_value(0.0)
  {
	if(name.find("th")!=std::string::npos )
	{
		//fills the vector of joint names: we're taking J2 and J1 for TH
		std::string tmp = boost::to_upper_copy(name);
		tmp += "J2";
		names_joints_linked.push_back(tmp);
		tmp = boost::to_upper_copy(name);
		tmp += "J1";
		names_joints_linked.push_back(tmp);
	}
	else
	{
		//fills the vector of joint names: we're taking J3 and J0
		std::string tmp = boost::to_upper_copy(name);
		tmp += "J3";
		names_joints_linked.push_back(tmp);
		tmp = boost::to_upper_copy(name);
		tmp += "J0";
		names_joints_linked.push_back(tmp);
	}

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

/**********************************
 *     TACTILE SENSOR MANAGER     *
 **********************************/
  SrVirtualTactileSensorManager::SrVirtualTactileSensorManager() :
    SrTactileSensorManager()
  {
    std::vector<std::vector<std::string> > all_names = get_all_names();

    for( unsigned int i=0; i< all_names[0].size() ; ++i)
    {
      tactile_sensors.push_back(
        boost::shared_ptr<SrVirtualTactileSensor>(
          new SrVirtualTactileSensor(all_names[0][i],
                                     all_names[1][i]) ));
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
