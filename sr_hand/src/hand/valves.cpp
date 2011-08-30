/**
 * @file   valves.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Feb 9 14:56:10 2010
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
 * @brief  This is a ROS Interface used to directly access the valves
 * on Shadow Robot's muscle robots.
 *
 */

#include "sr_hand/hand/valves.h"
//our robot code

#include <sstream>
//messages


namespace shadowrobot
{
  Valves::Valves() :
    n_tilde("~"), publish_rate(0.0)
  {

    /* We need to attach the program to the robot, or fail if we cannot. */
    if (robot_init() < 0)
    {
      ROS_FATAL("Robot interface broken\n");
      ROS_BREAK();
    }

    /* We need to attach the program to the hand as well, or fail if we cannot. */
    if (hand_init() < 0)
    {
      ROS_FATAL("Arm interface broken\n");
      ROS_BREAK();
    }

    // set publish frequency
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 10.0);
    publish_rate = ros::Rate(publish_freq);

#ifdef HAND_MUSCLE
    for (unsigned int i = 0; i < START_OF_ARM; ++i)
    {
      init_subs_and_pubs(i);
    }
#endif
#ifdef ARM
    for (unsigned int i = START_OF_ARM; i < NUM_HAND_JOINTS; ++i)
    {
      init_subs_and_pubs(i);
    }
#endif
  }

  Valves::~Valves()
  {
  }

  void Valves::init_subs_and_pubs(int index_joint)
  {
    std::vector<std::string> subname(2);
    subname[0] = "Flex";
    subname[1] = "Ext";

    int subname_index = 0;

    std::string name = hand_joints[index_joint].joint_name;
    for (unsigned int j = 0; j < hand_joints[index_joint].num_actuators; ++j)
    {
      //! for the old hand the phalange 4 only used the spring and the Ext only
      subname_index = (hand_joints[index_joint].num_actuators == 1) ? 1 : j;

      //Pressure
      struct sensor s = hand_joints[index_joint].a.muscle.pressure[j];
      valves_sensors.push_back(s);
      std::stringstream ss_valve_name;
      ss_valve_name << hand_joints[index_joint].joint_name << "_"
                    << subname[subname_index] << "_Pressure";
      std::string valve_name = ss_valve_name.str();
      ROS_DEBUG("%s", valve_name.c_str());
      std::string topic = valve_name + "/status";
      valves_publishers.push_back(n_tilde.advertise<std_msgs::Float64> (topic, 2));
      topic = valve_name + "/cmd";
      valves_subscribers.push_back(n_tilde.subscribe<std_msgs::Float64>(topic, 10, boost::bind(&Valves::valve_command, this, _1, valves_sensors.size()-1)));

      //Pressure_Target
      ss_valve_name.str(std::string());
      s = hand_joints[index_joint].a.muscle.pressure_target[j];
      valves_sensors.push_back(s);
      ss_valve_name << hand_joints[index_joint].joint_name << "_"
                    << subname[subname_index] << "_Target";
      valve_name = ss_valve_name.str();
      ROS_DEBUG("%s", valve_name.c_str());
      topic = valve_name + "/status";
      valves_publishers.push_back(n_tilde.advertise<std_msgs::Float64> (topic, 2));
      topic = valve_name + "/cmd";
      valves_subscribers.push_back(n_tilde.subscribe<std_msgs::Float64>(topic, 10, boost::bind(&Valves::valve_command, this, _1, valves_sensors.size() - 1 )));

      //Muscles
      ss_valve_name.str(std::string());
      s = hand_joints[index_joint].a.muscle.muscles[j];
      valves_sensors.push_back(s);
      ss_valve_name << hand_joints[index_joint].joint_name << "_"
                    << subname[subname_index];
      valve_name = ss_valve_name.str();
      ROS_DEBUG("%s", valve_name.c_str());
      topic = valve_name + "/status";
      valves_publishers.push_back(n_tilde.advertise<std_msgs::Float64> (topic, 2));
      topic = valve_name + "/cmd";
      valves_subscribers.push_back(n_tilde.subscribe<std_msgs::Float64>(topic, 10, boost::bind(&Valves::valve_command, this, _1, valves_sensors.size()-1 )));

      //Fill
      ss_valve_name.str(std::string());
      s = hand_joints[index_joint].a.muscle.valves[j][FILL_VALVE];
      valves_sensors.push_back(s);
      ss_valve_name << hand_joints[index_joint].joint_name << "_"
                    << subname[subname_index] << "_Fill";
      valve_name = ss_valve_name.str();
      ROS_DEBUG("%s", valve_name.c_str());
      topic = valve_name + "/status";
      valves_publishers.push_back(n_tilde.advertise<std_msgs::Float64> (topic, 2));
      topic = valve_name + "/cmd";
      valves_subscribers.push_back(n_tilde.subscribe<std_msgs::Float64>(topic, 10, boost::bind(&Valves::valve_command, this, _1, valves_sensors.size()-1)));

      //Empty
      ss_valve_name.str(std::string());
      s = hand_joints[index_joint].a.muscle.valves[j][EMPTY_VALVE];
      valves_sensors.push_back(s);
      ss_valve_name << hand_joints[index_joint].joint_name << "_"
                    << subname[subname_index] << "_Empty";
      valve_name = ss_valve_name.str();
      ROS_DEBUG("%s", valve_name.c_str());
      topic = valve_name + "/status";
      valves_publishers.push_back(n_tilde.advertise<std_msgs::Float64> (topic, 2));
      topic = valve_name + "/cmd";
      valves_subscribers.push_back(n_tilde.subscribe<std_msgs::Float64>(topic, 10, boost::bind(&Valves::valve_command, this, _1, valves_sensors.size() -1)));
    }

  }


/**
 * callback function for the valves: send a command to a given valve.
 *
 * we have on subscriber per valve. The subscriber index corresponds to the
 * index_valve. From this index_valve you can get the valve sensor
 * from the valves_sensors vector.
 *
 * @param msg the msg containing a value to send to a valve controller.
 * @param index_valve the index of the valve in the valves_sensors vector.
 *
 * @return
 */
  void Valves::valve_command(const std_msgs::Float64ConstPtr& msg, int index_valve)
  {
    //@fixme: do some clipping on the value first?
    robot_update_sensor(&(valves_sensors[index_valve]), msg->data);
  }

  /**
   * Callback function for the periodic publishing: publishes the
   * data for each valve.
   **/
  void Valves::publish()
  {
    std_msgs::Float64 msg_valves;

    for( unsigned int i = 0; i < valves_sensors.size(); ++i )
    {
      msg_valves.data = robot_read_sensor(&(valves_sensors[i]));

      valves_publishers[i].publish(msg_valves);
    }

    ros::spinOnce();
    publish_rate.sleep();

  }
} //end namespace


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
