/**
 * @file   sr_gazebo_virtual_tactile_sensor.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com> updated by Ravin de Souza <rsouza@isr.ist.utl.pt>, Contact <contact@shadowrobot.com>
 * @date
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
 * @brief  This is the virtual implementation of the SrGenericTactileSensor that publishes
 * touch data obtained from the robot hand simulation in gazebo.
 *
 */

#include "sr_tactile_sensors/sr_gazebo_virtual_tactile_sensor.hpp"
#include <boost/algorithm/string.hpp>
#include <string>
#include <math.h>
#include <ros/ros.h>

namespace shadowrobot
{
/**********************************
 *         TACTILE SENSOR         *
 **********************************/
  SrGazeboVirtualTactileSensor::SrGazeboVirtualTactileSensor(std::string name,
                                                             std::string gazebo_bumper_topic) :
    SrGenericTactileSensor(name, ""),
    touch_value(0.0),
    touch_freshdata(false)
  {
    sub = nh.subscribe(gazebo_bumper_topic, 2, &SrGazeboVirtualTactileSensor::callback, this);
  }

  SrGazeboVirtualTactileSensor::~SrGazeboVirtualTactileSensor()
  {}

  void SrGazeboVirtualTactileSensor::callback(const gazebo_msgs::ContactsState& msg)
  {
    double tmp_value;
    ::geometry_msgs::Vector3 v;
    v.x=0;
    v.y=0;
    v.z=0;

    ROS_DEBUG("Touch by %s", msg.header.frame_id.c_str());
    // Parse the message to retrieve the relevant touch pressure information
    // Currently the sum of all contacts.
    // More sophisticated analysis can be done to take the contact that is the most aligned with the distal link normal

    if(msg.states.size()>0)
    {
      unsigned int nb_wrench = msg.states[0].wrenches.size();
      for (unsigned int i=0;i<nb_wrench;i++)
      {
          v.x=v.x+msg.states[0].wrenches[i].force.x;
          v.y=v.y+msg.states[0].wrenches[i].force.y;
          v.z=v.z+msg.states[0].wrenches[i].force.z;
      }
      v.x=v.x/nb_wrench;
      v.y=v.y/nb_wrench;
      v.z=v.z/nb_wrench;
      tmp_value = sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
      touch_mutex.lock();
      touch_value = tmp_value;
      touch_freshdata =  true;
      touch_mutex.unlock();
    }
  }

  double SrGazeboVirtualTactileSensor::get_touch_data()
  {
    double return_value;
    touch_mutex.lock();
    if(touch_freshdata)
    {
        return_value = touch_value;
        touch_freshdata=false;
    }
    else
    {
        return_value= 0.0;
    }
    touch_mutex.unlock();

    return return_value;
  }


/**********************************
 *     TACTILE SENSOR MANAGER     *
 **********************************/
  SrGazeboVirtualTactileSensorManager::SrGazeboVirtualTactileSensorManager() :
    SrTactileSensorManager()
  {
    std::vector<std::vector<std::string> > all_names = get_all_names();

    for( unsigned int i=0; i< all_names[0].size() ; ++i)
    {
      tactile_sensors.push_back(
        boost::shared_ptr<SrGazeboVirtualTactileSensor>(
          new SrGazeboVirtualTactileSensor(all_names[0][i],
                                           "contacts/"+all_names[0][i]+"/distal")
        )
      );
    }
  }

  SrGazeboVirtualTactileSensorManager::~SrGazeboVirtualTactileSensorManager()
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

  shadowrobot::SrGazeboVirtualTactileSensorManager tact_sens_mgr;

  while( ros::ok() )
    tact_sens_mgr.publish_all();

  return 0;
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
