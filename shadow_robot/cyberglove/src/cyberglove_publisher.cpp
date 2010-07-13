/**
* @file   shadowhand_publisher.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>
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

//generic C/C++ include
#include <string>
#include <sstream>

#include "cyberglove/serial_glove.h"
#include "cyberglove/cyberglove_publisher.h"

using namespace ros;
using namespace xml_calibration_parser;

namespace cyberglove_publisher{

  /////////////////////////////////
  //    CONSTRUCTOR/DESTRUCTOR   //
  /////////////////////////////////

  CyberglovePublisher::CyberglovePublisher()
    : n_tilde("~"), publish_rate(0.0), path_to_glove("/dev/ttyS0"), publishing(true)
  {

    std::string path_to_calibration;
    n_tilde.param("path_to_calibration", path_to_calibration, std::string("/etc/robot/calibration.d/cyberglove.cal"));
    ROS_INFO("Calibration file loaded for the Cyberglove: %s", path_to_calibration.c_str());

    initialize_calibration(path_to_calibration);

    // set publish frequency
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 20.0);
    publish_rate = Rate(publish_freq);

    // set path to glove
    n_tilde.param("path_to_glove", path_to_glove, std::string("/dev/ttyS0"));
    ROS_INFO("Opening glove on port: %s", path_to_glove.c_str());

    int error = setup_glove( path_to_glove.c_str() );
    //sleep 1s to be sure the glove had enough time to start 
    sleep(1);

    if( error != 0 )
      ROS_ERROR("Couldn't initialize the glove, is the glove plugged in?");
    else
      {
	//publishes calibrated JointState messages
	std::string prefix;
	std::string searched_param;
	n_tilde.searchParam("cyberglove_prefix", searched_param);
	n_tilde.param(searched_param, prefix, std::string());
	std::string full_topic = prefix + "/calibrated/joint_states";
	cyberglove_pub = n_tilde.advertise<sensor_msgs::JointState>(full_topic, 2);

	//publishes raw JointState messages
	n_tilde.searchParam("cyberglove_prefix", searched_param);
	n_tilde.param(searched_param, prefix, std::string());
	full_topic = prefix + "/raw/joint_states";
	cyberglove_raw_pub = n_tilde.advertise<sensor_msgs::JointState>(full_topic, 2);
      }

    //initialises joint names (the order is important)
    jointstate_msg.name.push_back("G_ThumbRotate");
    jointstate_msg.name.push_back("G_ThumbMPJ");
    jointstate_msg.name.push_back("G_ThumbIJ");
    jointstate_msg.name.push_back("G_ThumbAb");
    jointstate_msg.name.push_back("G_IndexMPJ");
    jointstate_msg.name.push_back("G_IndexPIJ");
    jointstate_msg.name.push_back("G_IndexDIJ");
    jointstate_msg.name.push_back("G_MiddleMPJ");
    jointstate_msg.name.push_back("G_MiddlePIJ");
    jointstate_msg.name.push_back("G_MiddleDIJ");
    jointstate_msg.name.push_back("G_MiddleIndexAb");
    jointstate_msg.name.push_back("G_RingMPJ");
    jointstate_msg.name.push_back("G_RingPIJ");
    jointstate_msg.name.push_back("G_RingDIJ");
    jointstate_msg.name.push_back("G_RingMiddleAb");
    jointstate_msg.name.push_back("G_PinkieMPJ");
    jointstate_msg.name.push_back("G_PinkiePIJ");
    jointstate_msg.name.push_back("G_PinkieDIJ");
    jointstate_msg.name.push_back("G_PinkieRingAb");
    jointstate_msg.name.push_back("G_PalmArch");
    jointstate_msg.name.push_back("G_WristPitch");
    jointstate_msg.name.push_back("G_WristYaw");

    jointstate_raw_msg.name = jointstate_msg.name;
  }

  CyberglovePublisher::~CyberglovePublisher()
  {
  }
  
  void CyberglovePublisher::initialize_calibration(std::string path_to_calibration)
  {
    calibration_parser = XmlCalibrationParser(path_to_calibration);
  }

  bool CyberglovePublisher::isPublishing()
  {
    if (publishing)
      {
	return true;
      }
    else
      {
	//check if the value was read
	if( checkGloveState() )
	  {
	    ROS_INFO("The glove button was switched on, starting to publish data.");
	    publishing = true;
	  }
	
	ros::spinOnce();
	publish_rate.sleep();
	return false;
      }
  }

  void CyberglovePublisher::setPublishing(bool value){
    publishing = value;
  }

  /////////////////////////////////
  //       PUBLISH METHOD        //
  /////////////////////////////////
  void CyberglovePublisher::publish()
  {
    //if (!publishing) return;
    //read the state of the glove button
    if( !checkGloveState() )
      {
	publishing = false;
	ROS_INFO("The glove button is off, no data will be read / sent");
	ros::spinOnce();
	publish_rate.sleep();
	return;
      }

    //read data from the glove
    try
      {
	glovePositions = glove_get_values();
      }
    catch(int e)
      {
	ROS_ERROR("The glove values can't be read");
	ros::spinOnce();
	publish_rate.sleep();
	return;
      }

    //reset the messages
    jointstate_msg.effort.clear();
    jointstate_msg.position.clear();
    jointstate_msg.velocity.clear();
    jointstate_raw_msg.effort.clear();
    jointstate_raw_msg.position.clear();
    jointstate_raw_msg.velocity.clear();

    //fill the joint_state msg with the glove data
    for(unsigned int i=0; i<GLOVE_SIZE; ++i)
      {
	jointstate_raw_msg.position.push_back(glovePositions[i]);
	add_jointstate(glovePositions[i], jointstate_msg.name[i]);
      }
    //publish the msgs
    cyberglove_pub.publish(jointstate_msg);
    cyberglove_raw_pub.publish(jointstate_raw_msg);

    ros::spinOnce();
    publish_rate.sleep();
  }

  void CyberglovePublisher::add_jointstate(float position, std::string joint_name)
  {
    //can't read the effort from the glove
    jointstate_msg.effort.push_back(0.0);

    //get the calibration value
    float calibration_value = calibration_parser.get_calibration_value(position, joint_name);
    std::cout << calibration_value << std::endl;
    //publish the glove position
    jointstate_msg.position.push_back(calibration_value);
    //set velocity to 0. 
    //@TODO : send the correct velocity ?
    jointstate_msg.velocity.push_back(0.0);
  }

  bool CyberglovePublisher::checkGloveState()
  {
    int gloveButtonState = -1;
    gloveButtonState =  read_button_value();
    
    //check if the value was read
    switch( gloveButtonState)
      {
      case 0:
	return false;
      case 1:
	return true;
      default:
	ROS_ERROR("The glove button state value couldn't be read.");
	return false;
      }
  }

}// end namespace


