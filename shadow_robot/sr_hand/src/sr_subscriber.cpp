/**
 * @file   sr_subscriber.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 25 15:34:37 2010
 * 
 * @brief  The role of this ROS subscriber is to receive commands
 * messages (sent by a publisher) and pass them to the hand after
 * translating them to the correct format. 
 * 
 * 
 */

#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_datatypes.h>

#include "sr_hand/sr_subscriber.h"

using namespace std;


namespace shadowhand_subscriber {

  /////////////////////////////////
  //    CONSTRUCTOR/DESTRUCTOR   //
  /////////////////////////////////

  ShadowhandSubscriber::ShadowhandSubscriber(boost::shared_ptr<Shadowhand> sh)
    : n_tilde("~")
  {
    shadowhand = sh;

    ///////  
    // Initialize the subscribers
    //////
    ShadowhandSubscriber::init();
  }

  ShadowhandSubscriber::ShadowhandSubscriber(boost::shared_ptr<Shadowhand> sh, KDL::Tree tree)
    : n_tilde("~")    
  {
    shadowhand = sh;

    sr_kinematics = boost::shared_ptr<SrKinematics>(new SrKinematics(tree));

    ///////
    // Initialize the subscribers
    //////
    ShadowhandSubscriber::init();

    // subscribe to reverseKinematics topic, set
    // reverseKinematicsCallback function
    std::string prefix;
    std::string searched_param;
    n_tilde.searchParam("shadowhand_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic = "/tf";
    reverse_kinematics_sub = node.subscribe(full_topic, 10, &ShadowhandSubscriber::reverseKinematicsCallback, this);

    joints_map = shadowhand->getAllJointsData();
    for(Shadowhand::JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it)
      {
	current_angles.push_back(it->second.position);
      }
  }


  ShadowhandSubscriber:: ~ShadowhandSubscriber()
  {
    //    if( shadowhand != NULL )
    //      delete shadowhand;
  }


  void ShadowhandSubscriber::init()
  {
    // subscribe to sendupdate topic, set sendupdateCallback function
    std::string prefix;
    std::string searched_param;
    n_tilde.searchParam("shadowhand_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "sendupdate";

    sendupdate_sub = node.subscribe(full_topic, 2, &ShadowhandSubscriber::sendupdateCallback, this);

    // subscribe to contrlr topic, set contrlrCallback function
    full_topic = prefix + "contrlr";
    contrlr_sub = node.subscribe(full_topic, 2,  &ShadowhandSubscriber::contrlrCallback, this);

    // subscribe to config topic, set configCallback function
    full_topic = prefix + "config";
    config_sub = node.subscribe(full_topic, 2, &ShadowhandSubscriber::configCallback, this);
  }


  
  /////////////////////////////////
  //         CALLBACK            //
  /////////////////////////////////
  void ShadowhandSubscriber::sendupdateCallback(const sr_hand::sendupdateConstPtr& msg)
  {    
    //loop on all the sendupdate messages received (if > 0)
    int sendupdate_length = msg->sendupdate_length;
    if( sendupdate_length == 0)
      {
	ROS_WARN("Received empty sendupdate command.");
	return;
      }
    //OK, not empty => loop to process all the sendupdate messages
    for(unsigned short index_msg=0; index_msg < msg->sendupdate_length; ++index_msg)
      {
	float target = msg->sendupdate_list[index_msg].joint_target;
	string sensor_name = msg->sendupdate_list[index_msg].joint_name;
      
	ROS_DEBUG("Received sendupdate Command [%s : %f]", sensor_name.c_str(), target);
	shadowhand->sendupdate(sensor_name, (double)target);
      }

  }
  

  void ShadowhandSubscriber::contrlrCallback(const sr_hand::contrlrConstPtr& msg)
  {
    
    vector<string> list_of_parameters = msg->list_of_parameters;

    JointControllerData ctrl_data;

    //parses all the parameters transmitted in the msg
    for (unsigned int index_param = 0; index_param < msg->length_of_list; ++index_param)
      {
	//split the string (around ":")
	vector<string> splitted_string;
	boost::split(splitted_string, msg->list_of_parameters[index_param], boost::is_any_of(":"));
      
	Parameters param;
	param.name  = splitted_string[0];
	param.value = splitted_string[1];	

	ctrl_data.data.push_back(param);
      }

    shadowhand->setContrl(msg->contrlr_name, ctrl_data);
  }

  void ShadowhandSubscriber::configCallback(const sr_hand::configConstPtr& msg)
  { 
    ROS_ERROR("Configuration command callback not implemented yet");
  }

  
  void ShadowhandSubscriber::reverseKinematicsCallback( const tf::tfMessageConstPtr& msg )
  {
    tf::StampedTransform transform;
    try
      {
	tf_listener.lookupTransform("threedmouse", "sr_arm/position/shadowarm_handsupport", ros::Time(0), transform);

	//read current joint positions from the hand
	joints_map = shadowhand->getAllJointsData();
	int index = 0;

	for(Shadowhand::JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it)
	  {
	    //ROS_ERROR("pos: %f",it->second.position);
	    current_angles[index] = (it->second.position);
	    ++index;
	  }

	//compute the reverse kinematics
	sr_kinematics->computeReverseKinematics(transform, current_angles);



      }
    catch(tf::TransformException ex)
      {
	ROS_DEBUG("%s",ex.what());
      }
  }

} // end namespace
