/**
 * @file   cyberglove_publisher.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:25:55 2010
 * 
 * @brief  This class reads and publishes data concerning the
 * cyberglove. To publish those data, just call the publish()
 * function. 
 * 
 * 
 */

#ifndef   	CYBERGLOVE_PUBLISHER_H_
# define   	CYBERGLOVE_PUBLISHER_H_

#include <ros/ros.h>
#include <vector>

//messages
#include <sensor_msgs/JointState.h>
#include "cyberglove/xml_calibration_parser.h"

using namespace ros;

namespace cyberglove_publisher{

class CyberglovePublisher
{
 public:
  /// Constructor
  CyberglovePublisher();
  
  /// Destructor
  ~CyberglovePublisher(){};

  Publisher cyberglove_pub;
  bool isPublishing;
  void initialize_calibration(std::string path_to_calibration);
  void publish();
  
 private:
  /////////////////
  //  CALLBACKS  //
  /////////////////

  //ros node handle
  NodeHandle node, n_tilde;
  Rate publish_rate;
  std::string path_to_glove;

  ///the calibration parser
  xml_calibration_parser::XmlCalibrationParser calibration_parser;

  Publisher cyberglove_raw_pub;

  sensor_msgs::JointState jointstate_msg;
  sensor_msgs::JointState jointstate_raw_msg;

  void add_jointstate(float position, std::string joint_name);

  std::vector<float> calibration_values;
}; // end class CyberglovePublisher

} // end namespace
#endif 	    /* !CYBERGLOVE_PUBLISHER_H_ */
