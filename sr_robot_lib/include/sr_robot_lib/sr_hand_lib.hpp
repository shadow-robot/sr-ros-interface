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
#include <std_srvs/Empty.h>

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

    /**
     * The service callback for setting the Force PID values. There's only one callback
     * function, but it can called for any motors. We know which motor called the service
     * thanks to the motor_index.
     *
     * @param request The request contains the new parameters for the controllers.
     * @param response True if succeeded.
     * @param motor_index The index of the motor for which the service has been called.
     *
     * @return true if succeeded.
     */
    bool force_pid_callback(sr_robot_msgs::ForceController::Request& request,
                            sr_robot_msgs::ForceController::Response& response,
                            int motor_index);

    /**
     * Reset the motor at motor index.
     *
     * @param request empty
     * @param response empty
     * @param joint A pair containing the index of the motor for the given
     *              joint followed by the name of the joint we're resetting
     *
     * @return true if success
     */
    bool reset_motor_callback(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response,
                              std::pair<int,std::string> joint);

#ifdef DEBUG_PUBLISHER
    /**
     * This is a service callback: we set the debug data we want to publish
     * at full speed in the debug topics.
     *
     * @param request Contains the motor index and the MOTOR_DATA type
     * @param response True if succeedded.
     *
     * @return true if succeeded.
     */
    bool set_debug_data_to_publish(sr_robot_msgs::SetDebugData::Request& request,
                                   sr_robot_msgs::SetDebugData::Response& response);
#endif

  protected:
    /**
     * Initializes the hand library with the needed values.
     *
     * @param joint_names A vector containing all the joint names.
     * @param motor_ids A vector containing the corresponding motor ids.
     * @param joint_to_sensors A vector mapping the joint to the sensor index we read from the palm.
     * @param actuators A vector containing the actuators for the different joints.
     */
    virtual void initialize(std::vector<std::string> joint_names, std::vector<int> motor_ids,
                            std::vector<shadow_joints::JointToSensor> joint_to_sensors,
                            std::vector<sr_actuator::SrActuator*> actuators);

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

    static const int nb_motor_data;
    static const char* human_readable_motor_data_types[];
    static const FROM_MOTOR_DATA_TYPE motor_data_types[];

    /// a service server for reconfiguring the debug data we want to publish
    ros::ServiceServer debug_service;

    /**
     * Read the motor board force pids from the parameter servers,
     * called when resetting the motor.
     *
     * @param joint_name the joint we want to reset
     */
    void resend_pids(std::string joint_name);
  };

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
