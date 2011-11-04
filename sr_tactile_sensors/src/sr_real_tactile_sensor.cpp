/**
 * @file   sr_real_tactile_sensor.cpp
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
 * @brief  This is the implementation of the SrGenericTactileSensor. It
 * fetches data from the real sensors.
 *
 *
 */

#include "sr_tactile_sensors/sr_real_tactile_sensor.hpp"

namespace shadowrobot
{
/**********************************
 *         TACTILE SENSOR         *
 **********************************/
  SrRealTactileSensor::SrRealTactileSensor(std::string name,
                                           std::string touch_name ) :
    SrGenericTactileSensor(name, touch_name)
  {
    /* We need to attach the program to the robot, or fail if we cannot. */
    if (robot_init() < 0)
    {
      ROS_FATAL("Robot interface broken\n");
      ROS_BREAK();
    }

    res_touch = robot_name_to_sensor(touch_name.c_str(), &sensor_touch);

    if(res_touch)
    {
      ROS_ERROR("Can't open sensor %s", touch_name.c_str());
    }
  }

  SrRealTactileSensor::~SrRealTactileSensor()
  {}

  double SrRealTactileSensor::get_touch_data()
  {
    if(res_touch)
      return -1000.0;

    return robot_read_sensor(&sensor_touch);
  }


/**********************************
 *     TACTILE SENSOR MANAGER     *
 **********************************/
  SrRealTactileSensorManager::SrRealTactileSensorManager() :
    SrTactileSensorManager()
  {
    std::vector<std::vector<std::string> > all_names = get_all_names();

    for( unsigned int i=0; i < all_names[0].size() ; ++i)
    {
      tactile_sensors.push_back(
        boost::shared_ptr<SrRealTactileSensor>(
          new SrRealTactileSensor(all_names[0][i],
                                  all_names[1][i]) ));
    }
  }

  SrRealTactileSensorManager::~SrRealTactileSensorManager()
  {}
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

  shadowrobot::SrRealTactileSensorManager tact_sens_mgr;

  while( ros::ok() )
    tact_sens_mgr.publish_all();

  return 0;
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
