/**
 * @file   sr_hand_lib.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Jun  3 12:12:13 2011
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
 * @brief This is a library for the etherCAT hand.
 * You can find it instantiated in the sr_edc_ethercat_drivers.
 *
 *
 */

#ifndef _SR_HAND_LIB_HPP_
#define _SR_HAND_LIB_HPP_

#include "sr_robot_lib/sr_robot_lib.hpp"

//to be able to load the configuration from the
//parameter server
#include <ros/ros.h>

namespace shadow_robot
{
  class SrHandLib : public SrRobotLib
  {
  public:
    SrHandLib(pr2_hardware_interface::HardwareInterface *hw);
    ~SrHandLib();


    bool force_pid_callback(sr_robot_msgs::ForceController::Request& request, sr_robot_msgs::ForceController::Response& response, int motor_index);



  protected:
    virtual void initialize(std::vector<std::string> joint_names, std::vector<int> motor_ids,
                            std::vector<shadow_joints::JointToSensor> joint_to_sensors,
                            std::vector<pr2_hardware_interface::Actuator*> actuators);

  private:
    ros::NodeHandle nodehandle_;

    /**
     * Reads the mapping between the sensors and the joints from the parameter server.
     *
     *
     * @return a vector (size of the number of joints) containing vectors (containing
     *         the sensors which are combined to form a given joint)
     */
    std::vector<shadow_joints::JointToSensor> read_joint_to_sensor_mapping();

    /**
     * Reads the calibration from the parameter server.
     *
     *
     * @return a calibration map
     */
    shadow_joints::CalibrationMap read_joint_calibration();

    /**
     * Reads the mapping associating a joint to a motor.
     * If the motor index is -1, then no motor is associated
     * to this joint.
     *
     *
     * @return a vector of motor indexes, ordered by joint.
     */
    std::vector<int> read_joint_to_motor_mapping();

    /**
     * Simply reads the config from the parameter server.
     *
     * @return A vector of UpdateConfig containing the type of data and the frequency
     *         at which we want to poll this data
     */
    std::vector<motor_updater::UpdateConfig> read_update_rate_configs();
  };

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
