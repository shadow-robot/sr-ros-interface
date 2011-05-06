/**
* @file   shadowhand_publisher.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>
* @date   Thu Mar 25 15:36:41 2010
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
* @brief The goal of this ROS publisher is to publish raw and calibrated
* joint positions from the cyberglove at a regular time interval. We're
* oversampling to get a better accuracy on our data.
*
*
*/

//ROS include
#include <ros/ros.h>

//generic C/C++ include
#include <string>
#include <sstream>

#include "cyberglove/cyberglove_publisher.h"

using namespace ros;
using namespace xml_calibration_parser;

namespace cyberglove{

  /////////////////////////////////
  //    CONSTRUCTOR/DESTRUCTOR   //
  /////////////////////////////////

  CyberglovePublisher::CyberglovePublisher()
    : n_tilde("~"), sampling_rate(0.0), publish_counter_max(0), publish_counter_index(0),
      path_to_glove("/dev/ttyS0"), publishing(true)
  {

    std::string path_to_calibration;
    n_tilde.param("path_to_calibration", path_to_calibration, std::string("/etc/robot/calibration.d/cyberglove.cal"));
    ROS_INFO("Calibration file loaded for the Cyberglove: %s", path_to_calibration.c_str());

    initialize_calibration(path_to_calibration);

    //set sampling frequency
    double sampling_freq;
    n_tilde.param("sampling_frequency", sampling_freq, 100.0);
    sampling_rate = Rate(sampling_freq);

    // set publish_counter: the number of data we'll average
    // before publishing.
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 20.0);
    publish_counter_max = (int)(sampling_freq / publish_freq);

    ROS_INFO_STREAM("Sampling at " << sampling_freq << "Hz ; Publishing at "
                    << publish_freq << "Hz ; Publish counter: "<< publish_counter_max);

    // set path to glove
    n_tilde.param("path_to_glove", path_to_glove, std::string("/dev/ttyS0"));
    ROS_INFO("Opening glove on port: %s", path_to_glove.c_str());

    //initialize the connection with the cyberglove and binds the callback function
    serial_glove = boost::shared_ptr<CybergloveSerial>(new CybergloveSerial(path_to_glove, boost::bind(&CyberglovePublisher::glove_callback, this, _1, _2)));

    int res = -1;
    cyberglove_freq::CybergloveFreq frequency;

    switch( (int)sampling_freq)
    {
    case 100:
      res = serial_glove->set_frequency(frequency.hundred_hz);
      break;
    case 45:
      res = serial_glove->set_frequency(frequency.fourtyfive_hz);
      break;
    case 10:
      res = serial_glove->set_frequency(frequency.ten_hz);
      break;
    case 1:
      res = serial_glove->set_frequency(frequency.one_hz);
      break;
    default:
      res = serial_glove->set_frequency(frequency.hundred_hz);
      break;
    }
    //No filtering: we're oversampling the data, we want a fast poling rate
    res = serial_glove->set_filtering(false);
    //We want the glove to transmit the status (light on/off)
    res = serial_glove->set_transmit_info(true);
    //start reading the data.
    res = serial_glove->start_stream();

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
      return false;
    }
  }

  void CyberglovePublisher::setPublishing(bool value)
  {
    publishing = value;
  }

  /////////////////////////////////
  //       CALLBACK METHOD       //
  /////////////////////////////////
  void CyberglovePublisher::glove_callback(std::vector<float> glove_pos, bool light_on)
  {
    //if the light is off, we don't publish any data.
    if( !light_on )
    {
      publishing = false;
      ROS_DEBUG("The glove button is off, no data will be read / sent");
      ros::spinOnce();
      sampling_rate.sleep();
      return;
    }
    publishing = true;

    //appends the current position to the vector of position
    glove_positions.push_back( glove_pos );

    //if we've enough samples, publish the data:
    if( publish_counter_index == publish_counter_max )
    {
      //reset the messages
      jointstate_msg.position.clear();
      jointstate_msg.velocity.clear();
      jointstate_raw_msg.position.clear();
      jointstate_raw_msg.velocity.clear();

      //fill the joint_state msg with the averaged glove data
      for(unsigned int index_joint = 0; index_joint < CybergloveSerial::glove_size; ++index_joint)
      {
        //compute the average over the samples for the current joint
        float averaged_value = 0.0f;
        for (unsigned int index_sample = 0; index_sample < publish_counter_max; ++index_sample)
        {
          averaged_value += glove_positions[index_sample][index_joint];
        }
        averaged_value /= publish_counter_max;

        jointstate_raw_msg.position.push_back(averaged_value);
        add_jointstate(averaged_value, jointstate_msg.name[index_joint]);
      }

      //publish the msgs
      cyberglove_pub.publish(jointstate_msg);
      cyberglove_raw_pub.publish(jointstate_raw_msg);

      publish_counter_index = 0;
      glove_positions.clear();
    }

    publish_counter_index += 1;
    ros::spinOnce();
    sampling_rate.sleep();
  }

  void CyberglovePublisher::add_jointstate(float position, std::string joint_name)
  {
    //get the calibration value
    float calibration_value = calibration_parser.get_calibration_value(position, joint_name);
    //publish the glove position
    jointstate_msg.position.push_back(calibration_value);
    //set velocity to 0.
    //@TODO : send the correct velocity ?
    jointstate_msg.velocity.push_back(0.0);
  }
}// end namespace



/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
