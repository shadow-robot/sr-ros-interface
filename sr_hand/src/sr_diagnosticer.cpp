/**
* @file   sr_diagnosticer.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Thu Mar 25 15:36:41 2010
*
* @brief The goal of this ROS publisher is to publish relevant data
* concerning the hand at a regular time interval.
* Those data are (not exhaustive): positions, targets, temperatures,
* currents, forces, error flags, ...
*
*
*/

//ROS include
#include <ros/ros.h>

//messages
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

//generic C/C++ include
#include <vector>
#include <string>
#include <sstream>

#include <boost/smart_ptr.hpp>

#include "sr_hand/sr_diagnosticer.h"

using namespace ros;
using namespace shadowhand;

namespace shadowhand_diagnosticer{


// 9 is the number of messages sent on the palm: used in
// converting rate to Hz
const double ShadowhandDiagnosticer::palm_numb_msg_const = 9.0;
const double ShadowhandDiagnosticer::palm_msg_rate_const = 4000.0;

/////////////////////////////////
//    CONSTRUCTOR/DESTRUCTOR   //
/////////////////////////////////

ShadowhandDiagnosticer::ShadowhandDiagnosticer(boost::shared_ptr<Shadowhand> sh, hardware_types hw_type)
: n_tilde("~"), publish_rate(0.0)
{
  shadowhand = sh;

  // set publish frequency
  double publish_freq;
  n_tilde.param("publish_frequency_diagnostics", publish_freq, 1.0);
  publish_rate = Rate(publish_freq);

  //publishes /diagnostics messages
  shadowhand_diagnostics_pub = node.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 2);

  hardware_type = hw_type;
}

ShadowhandDiagnosticer::~ShadowhandDiagnosticer()
{
  //if( shadowhand != NULL )
   // delete shadowhand;
}

/////////////////////////////////
//       PUBLISH METHOD        //
/////////////////////////////////
void ShadowhandDiagnosticer::publish()
{
  diagnostic_msgs::DiagnosticArray diag_msg;

  std::vector<diagnostic_msgs::DiagnosticStatus> vec_diag_msg;

  std::vector<DiagnosticData> diagnostics = shadowhand->getDiagnostics();

  std::stringstream ss;

  for(unsigned int i=0; i<diagnostics.size(); ++i)
  {
    diagnostic_msgs::DiagnosticStatus diag;

    std::vector<diagnostic_msgs::KeyValue> keyvalues;

    diag.level = diagnostics[i].level;

    switch (hardware_type)
    {
      case sr_hand_hardware:
        diag.name = "srh/" + diagnostics[i].joint_name;
        break;
      case sr_arm_hardware:
        diag.name = "sr_arm/" + diagnostics[i].joint_name;
        break;
      default:
        diag.name = diagnostics[i].joint_name;
        break;
    }

    diagnostic_msgs::KeyValue keyval;
    keyval.key = "Target";

    ss.str("");
    ss << diagnostics[i].target;
    keyval.value = ss.str();
    keyvalues.push_back(keyval);

    keyval.key = "Position";
    ss.str("");
    ss << diagnostics[i].position;
    keyval.value = ss.str();
    keyvalues.push_back(keyval);

    if(diag.level == 0)
      diag.message = "OK";

    diag.values = keyvalues;
    vec_diag_msg.push_back(diag);
  }

  //set the standard message
  diag_msg.status = vec_diag_msg;

  /*  //get the data from the hand
      for (unsigned int i=0; i<NUM_HAND_JOINTS + 4; ++i)
      {
	diagnostic_msgs::DiagnosticStatus diag;
	std::vector<diagnostic_msgs::KeyValue> keyvalues;

	//set level to OK at first, will be updated if we get a flag
	//later on.
	diag.level = 0;

	//name
	std::stringstream name;
	name << "shadowhand/motors/";
	name << hand_joints[i].joint_name;
	diag.name = name.str();

	std::stringstream ss;
	std::stringstream warning_summary;


	//more information
	if (hand_joints[i].a.smartmotor.has_sensors) 
	  {
	    //check for error_flags
	    uint64_t uuid = robot_node_id(hand_joints[i].a.smartmotor.nodename);

	    char name_buff[40];
	    snprintf(name_buff, 40, "%Lx", uuid);
	    diag.hardware_id = name_buff;

	    hand_protocol_flags fl;
	    fl = hand_protocol_get_status_flags(uuid);

	    hand_protocol_config_t hand_config;
	    hand_config = hand_protocol_get_config(uuid);

	    //stringstream used everywhere to set strings...
	    std::stringstream tmp;

	    diagnostic_msgs::KeyValue keyval;	  
	    if (fl.valid) 
	      {
		struct hand_protocol_flags_smart_motor f;
		f = fl.u.smart_motor;

		//////////
		// ERRORS
		/////////
		if( hand_protocol_dead(uuid) )
		  {
		    warning_summary << "DEAD ";
		    diag.level = 2;

		    keyval.key = "DEAD";
		    tmp.str("");
		    tmp << "The node is DEAD (failed to pong back).";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }


		if( f.nfault_pin )
		  {
		    warning_summary << "NFAULT ";

		    diag.level = 2;


		    keyval.key = "NFAULT";
		    tmp.str("");
		    tmp << "The motor is in NFAULT state ";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }



		//////////
		// SOME GENERIC WARNINGS
		/////////

		if( f.mode_active )
		  {	  
		    warning_summary << "IDLE ";

		    diag.level > 1 ? diag.level = diag.level : diag.level = 1;


		    keyval.key = "IDLE";
		    tmp.str("");
		    tmp << "The motor is in IDLE state";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }      
		if( f.mode_config )
		  {
		    warning_summary << "CONFIG ";

		    diag.level > 1 ? diag.level = diag.level : diag.level = 1;


		    keyval.key = "CONFIG";
		    tmp.str("");
		    tmp << "The motor is in CONFIG mode";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }     


		//////////
		// SETPOINT
		/////////
		keyval.key = "- Setpoints";
		tmp.str(" ");
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//what's the setpoint set to?
		keyval.key = "  Setpoint";
		tmp.str("");
		uint16_t sensorId = hand_config->u.smartmotor.setpoint_num;
		tmp << get_setpoint_name(sensorId);
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//is there a SETPOINT error?
		if( f.setp_valid )
		  {
		    warning_summary << "SETPOINT ";

		    diag.level > 1 ? diag.level = diag.level : diag.level = 1;


		    keyval.key = "  SETPOINT";
		    tmp.str("");
		    tmp << "The SETPOINT is invalid";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }   

		//read the actual value of the target
		keyval.key = "  Target";
		tmp.str("");	 
		tmp << robot_read_sensor(&hand_joints[i].joint_target);
		keyval.value = tmp.str();
		keyvalues.push_back( keyval ); 

		//read value of setpoint (to compare with actual target)
		struct sensor temp;
		keyval.key = "  Target from setpoint";
		tmp.str("");	 
		robot_channel_to_sensor("smart_motor_setpoints", sensorId, &temp);
		tmp << robot_read_sensor(&temp);
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);


		//////////
		// SENSOR
		/////////
		keyval.key = "- Sensors";
		tmp.str(" ");
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//what's the sensor set to?
		keyval.key = "  Sensor";
		tmp.str("");
		sensorId = hand_config->u.smartmotor.sensor_num;

		tmp << get_setpoint_name(sensorId);
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//is there a SENSOR error?
		if( f.sensor_valid )
		  {
		    warning_summary << "SENSOR ";

		    diag.level > 1 ? diag.level = diag.level : diag.level = 1;


		    keyval.key = "  SENSOR";
		    tmp.str("");
		    tmp << "The SENSOR is invalid";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }   

		//read the actual value of the position
		keyval.key = "  Position";
		tmp.str("");	 
		tmp << robot_read_sensor(&hand_joints[i].position);
		keyval.value = tmp.str();
		keyvalues.push_back( keyval ); 

		//read value of sensor (to compare with actual position)
		keyval.key = "  Position from sensor";
		tmp.str("");	 
		robot_channel_to_sensor("smart_motor_setpoints", sensorId, &temp);
		tmp << robot_read_sensor(&temp);
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//////////
		// TEMPERATURE
		/////////
		keyval.key = "- Temperatures";
		tmp.str(" ");
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//value of the temperature
		keyval.key = "  temperature";
		float tempVal = robot_read_sensor(&hand_joints[i].a.smartmotor.temperature);
		tmp.str("");
		tmp << tempVal;
		keyval.value = tmp.str();
		keyvalues.push_back( keyval );

		//Is there a cutout?
		if( f.temperature_cutout )
		  {
		    warning_summary << "TEMP ";

		    diag.level > 1 ? diag.level = diag.level : diag.level = 1;


		    keyval.key = "  TEMPERATURE";
		    tmp.str("");
		    tmp << "The motor reached it's TEMPERATURE cutout value";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }

		//value of the temperature cutout?
		keyval.key = "  max temp";
		tmp.str("");
		tmp << ((float)(hand_config->u.smartmotor.max_motor_temperature)/256.0f);	  
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//////////
		// CURRENT
		/////////
		keyval.key = "- Currents";
		tmp.str(" ");
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		keyval.key = "  current";

		char incoming_current[40];
		snprintf(incoming_current, 40, "%Ld mA",  robot_read_incoming(&hand_joints[i].a.smartmotor.current));
		keyval.value = incoming_current;
		keyvalues.push_back( keyval );

		if( f.current_throttle )
		  {
		    warning_summary << "CURRENT ";

		    diag.level > 1 ? diag.level = diag.level : diag.level = 1;


		    keyval.key = "  CURRENT";
		    tmp.str("");
		    tmp << "The motor reached it's CURRENT cutout value";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }

		//value of the temperature cutout?
		keyval.key = "  max current";
		tmp.str("");
		tmp << (int)(hand_config->u.smartmotor.max_motor_current);
		tmp << "mA";
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//////////
		// FORCE
		/////////
		keyval.key = "- Forces";
		tmp.str(" ");
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		keyval.key = "  force";
		char incoming_force[40];
		int16_t torque = (int16_t)robot_read_incoming(&hand_joints[i].a.smartmotor.torque);

		//	      if(torque > 32767)
		//	torque -= 65536;
		int torque_32 =- torque;
		snprintf(incoming_force, 40, "%d", torque_32);  
		keyval.value = incoming_force;
		keyvalues.push_back( keyval );

		if( f.force_hard_limit )
		  {
		    warning_summary << "FORCE ";

		    diag.level > 1 ? diag.level = diag.level : diag.level = 1;


		    keyval.key = "  FORCE";
		    tmp.str("");
		    tmp << "The motor reached it's FORCE cutout value";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }

		//value of the force cutout?
		keyval.key = "  max force";
		tmp.str("");
		tmp << (int)(hand_config->u.smartmotor.safety_cutout_force);	  
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		if( f.straingauge_ref_limit0 )
		  {
		    warning_summary << "REF0 LIMIT ";

		    diag.level > 1 ? diag.level = diag.level : diag.level = 1;


		    keyval.key = "  REF0_LIMIT";
		    tmp.str("");
		    tmp << "The straingauge0 is at its limit.";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }
		if( f.straingauge_ref_limit1 )
		  {
		    warning_summary << "REF1 LIMIT ";

		    diag.level > 1 ? diag.level = diag.level : diag.level = 1;


		    keyval.key = "  REF1_LIMIT";
		    tmp.str("");
		    tmp << "The straingauge1 is at its limit.";
		    keyval.value = tmp.str();
		    keyvalues.push_back(keyval);
		  }


		//////////
		// Contrlr config
		/////////
		keyval.key = "- Controller Configuration";
		tmp.str(" ");
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//PID
		keyval.key = "  - PID";
		tmp.str(" ");
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		keyval.key = "    P | I | D values";
		tmp.str("");
		tmp << (int)(hand_config->u.smartmotor.sensorP) << " | "
		    << (int)(hand_config->u.smartmotor.sensorI) << " | "
		    << (int)(hand_config->u.smartmotor.sensorD);
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		keyval.key = "    Imax | Deadband | Offset";
		tmp.str("");
		tmp << (int)(hand_config->u.smartmotor.sensorImax) << " | "
		    << (int)(hand_config->u.smartmotor.sensorDeadband) << " | "
		    << (int)(hand_config->u.smartmotor.sensorOffset);
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//force PID
		keyval.key = "  - Force PID";
		tmp.str(" ");
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		keyval.key = "    force: P | I | D values";
		tmp.str("");
		tmp << (int)(hand_config->u.smartmotor.forceP) << " | "
		    << (int)(hand_config->u.smartmotor.forceI) << " | "
		    << (int)(hand_config->u.smartmotor.forceD);
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		keyval.key = "    Imax | Deadband | Offset";
		tmp.str("");
		tmp << (int)(hand_config->u.smartmotor.forceImax) << " | "
		    << (int)(hand_config->u.smartmotor.forceDeadband) << " | "
		    << (int)(hand_config->u.smartmotor.forceOffset);
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//////////
		// Transmission rates
		/////////
		keyval.key = "- Transmission Rates";
		tmp.str(" ");
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);
		//TRANSMIT RATE
		keyval.key = "  transmission rate";
		tmp.str("");

		tmp << palm_msg_rate_const/(palm_numb_msg_const*(float)(hand_config->u.smartmotor.tx_freq)) << "Hz";	  
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);

		//DEBUG TRANSMIT RATE
		keyval.key = "  debug transmission rate";
		tmp.str("");
		tmp << palm_msg_rate_const/(palm_numb_msg_const*(float)(hand_config->u.smartmotor.debug_tx_freq)) << "Hz";
		keyval.value = tmp.str();
		keyvalues.push_back(keyval);
	      }
	  }

	switch(diag.level)
	  {
	  default:
	    ss << "OK";
	    break;
	  case 1:
	    ss << "WARNING - " << warning_summary.str();
	    break;
	  case 2:
	    ss << "ERROR - " << warning_summary.str();
	    break;
	  }

	//set the diagnostic message
	diag.message = ss.str();

	diag.values = keyvalues;
	myVector.push_back(diag);
      }

    //set the standard message
    msg.status = myVector;
   */
  //publish the diagnostic data

  diag_msg.header.stamp = ros::Time::now();
  shadowhand_diagnostics_pub.publish(diag_msg);

  ros::spinOnce();
  publish_rate.sleep();
}


}// end namespace


