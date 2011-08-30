/**
 * @file   shadowhand_to_cybergrasp_remapper.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu May 13 09:44:52 2010
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
 * @brief This program remapps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 *
 *
 */

//ROS include
#include <ros/ros.h>

//generic include
#include <string>

#include <unistd.h>
#include <math.h>

//own .h
#include "sr_remappers/shadowhand_to_cybergrasp_remapper.h"

using namespace ros;

namespace shadowhand_to_cybergrasp_remapper{

  ShadowhandToCybergraspRemapper::ShadowhandToCybergraspRemapper()
    : n_tilde("~"), publish_rate(0.0)
  {
    std::string searched_param;

    //load the calibration
    std::string path;
    n_tilde.searchParam("cybergrasp_calibration_path", searched_param);
    n_tilde.param(searched_param, path, std::string());

    calibration_parser = new CalibrationParser(path);

   //publish to cybergraspforces topic
    std::string prefix;
    n_tilde.searchParam("cybergrasp_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "cybergraspforces";

    shadowhand_cybergrasp_pub = node.advertise<cybergrasp::cybergraspforces>(full_topic, 20);

    //subscribe to joint_states topic
    n_tilde.searchParam("joint_states_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());

    full_topic = prefix + "joint_states";

    shadowhand_jointstates_sub = node.subscribe(full_topic, 10, &ShadowhandToCybergraspRemapper::jointstatesCallback, this);

   }

  ShadowhandToCybergraspRemapper::~ShadowhandToCybergraspRemapper()
  {
    if( calibration_parser )
      delete calibration_parser;
  }

  /** 
   * process the joint_states callback: get the force data from the
   * finger tips and stream it to the cybergrasp
   * 
   * @param msg the joint_states message
   */
  void ShadowhandToCybergraspRemapper::jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
  { 
    //read msg and remap the vector to the cybergrasp
    cybergrasp::cybergraspforces cybergrasp_msg;

    //std::vector<double> pos(5);
    /*std::vector<double> difference_target_position(24);
    for(unsigned int i = 0 ; i < difference_target_position.size() ; ++i)
      {
	//velocity contains the targets... not the proper way of doing that
	difference_target_position[i] = fabs(msg->position[i] - msg->velocity[i]);
      }
    */
    //    pos = calibration_parser->get_remapped_vector(difference_target_position);
    /*    pos = calibration_parser->get_remapped_vector(msg->effort);

    for( unsigned int i=0; i < pos.size(); ++i)
      {
	double tmp = fabs(pos[i]);

	ROS_ERROR("%f", tmp);
	if(tmp > 0.015)
	  {
	    ROS_ERROR("%d >0.015", i);
	    cybergrasp_msg.forces[i] = (tmp-0.01)*7.0;
	  }
	else
	  cybergrasp_msg.forces[i] = 0.0;
      }
    */

    double average = 0.0;

    for( unsigned int i=0; i < msg->effort.size(); ++i)
      {
	average = msg->effort[i]*10.0;
      }
    
    average /= msg->effort.size();

    average /= msg->effort.size();
    average *= 1000.0;

    for( unsigned int i=0; i < 5; ++i)
      {
	cybergrasp_msg.forces[i] = 0.01;
      }

    //return the max of the read values on the sensor
    cybergrasp_msg.forces[1] = (average - 0.016)*2.0;
    //send vector to cybergrasp
    shadowhand_cybergrasp_pub.publish(cybergrasp_msg);

    //    sleep(2);

  }
}//end namespace
