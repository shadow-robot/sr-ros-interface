/**
 * @file   cyberglove_publisher.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:25:55 2010
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
 * To publish those data, just call the publish()
 * function.
 *
 *
 */

#ifndef   	CYBERGLOVE_PUBLISHER_H_
# define   	CYBERGLOVE_PUBLISHER_H_

#include <ros/ros.h>
#include <vector>
#include <boost/smart_ptr.hpp>

#include "cyberglove/serial_glove.hpp"

//messages
#include <sensor_msgs/JointState.h>
#include "cyberglove/xml_calibration_parser.h"

using namespace ros;

namespace cyberglove{

  class CyberglovePublisher
  {
  public:
    /// Constructor
    CyberglovePublisher();

    /// Destructor
    ~CyberglovePublisher();

    Publisher cyberglove_pub;
    void initialize_calibration(std::string path_to_calibration);
    bool isPublishing();
    void setPublishing(bool value);
  private:
    /////////////////
    //  CALLBACKS  //
    /////////////////

    //ros node handle
    NodeHandle node, n_tilde;
    Rate sampling_rate;
    unsigned int publish_counter_max, publish_counter_index;

    ///the actual connection with the cyberglove is done here.
    boost::shared_ptr<CybergloveSerial> serial_glove;

    /**
     * The callback function: called each time a full message
     * is received. This function is bound to the serial_glove
     * object using boost::bind.
     *
     * @param glove_pos A vector containing the current raw joints positions.
     * @param light_on true if the light is on, false otherwise.
     */
    void glove_callback(std::vector<float> glove_pos, bool light_on);

    std::string path_to_glove;
    bool publishing;

    ///the calibration parser
    xml_calibration_parser::XmlCalibrationParser calibration_parser;

    Publisher cyberglove_raw_pub;

    sensor_msgs::JointState jointstate_msg;
    sensor_msgs::JointState jointstate_raw_msg;

    void add_jointstate(float position, std::string joint_name);

    std::vector<float> calibration_values;

    std::vector<std::vector<float> > glove_positions;
  }; // end class CyberglovePublisher

} // end namespace
#endif 	    /* !CYBERGLOVE_PUBLISHER_H_ */

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
