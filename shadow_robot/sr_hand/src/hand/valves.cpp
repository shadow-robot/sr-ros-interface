/**
 * @file   valves.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Feb 9 14:56:10 2010
 *
 * @brief  This is a ROS Interface used to directly access the valves 
 * on Shadow Robot's muscle robots.
 *
 */

#include "sr_hand/hand/valves.h"
//our robot code

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
    n_tilde.param("publish_frequency", publish_freq, 20.0);
    publish_rate = ros::Rate(publish_freq);

    //initialize the valves names: parse the config files?
    valves_names.push_back("ShoulderSwing_Flex_Fill");
    valves_names.push_back("ShoulderSwing_Flex_Empty");

    //then initializes the vector of robot sensors from those names
    for(unsigned int i=0; i<valves_names.size(); ++i)
    {
      struct sensor s;
      int res = robot_name_to_sensor(valves_names[i].c_str(), &s);
      if( res )
        ROS_FATAL("Can't open sensor %s", valves_names[i].c_str());

      valves_sensors.push_back(s);
    }


    for( unsigned int i=0; i<valves_names.size(); ++i)
    {
      std::string topic = valves_names[i] + "/status";
      valves_publishers.push_back(n_tilde.advertise<std_msgs::Float64> (topic, 2));

      topic = valves_names[i] + "/cmd";
      valves_subscribers.push_back(n_tilde.subscribe<std_msgs::Float64>(topic, 10, boost::bind(&Valves::valve_command, this, _1, i)));
    }

  }

  Valves::~Valves()
  {
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
    ROS_DEBUG_STREAM("Valve["<< index_valve
                     << "] "<< valves_names[index_valve]);

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
