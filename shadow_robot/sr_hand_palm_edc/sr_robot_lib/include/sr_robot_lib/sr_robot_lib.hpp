/**
 * @file   sr_robot_lib.hpp
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
 * @brief This is a generic robot library for Shadow Robot's Hardware.
 *
 *
 */

#ifndef _SR_ROBOT_LIB_HPP_
#define _SR_ROBOT_LIB_HPP_

#include <boost/smart_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread.hpp>
#include <vector>

#include <sr_utilities/thread_safe_map.hpp>

#include <pr2_hardware_interface/hardware_interface.h>

#include "sr_robot_lib/calibration.hpp"

#include <sr_external_depencies/types_for_external.h>
extern "C"
{
  #include <sr_external_depencies/0220_palm_edc/0220_palm_edc_ethercat_protocol.h>
  #include <sr_external_depencies/simplemotor-bootloader/bootloader.h>
}


namespace shadow_joints
{
  struct PartialJointToSensor
  {
    int sensor_id;
    double coeff;
  };

  struct JointToSensor
  {
    std::vector<std::string> sensor_names;
    std::vector<PartialJointToSensor> joint_to_sensor_vector;
    bool calibrate_after_combining_sensors;
  };

  struct Motor
  {
    //the position of the motor in the motor array
    // coming from the hardware
    int motor_id;

    //the position of the motor in the message array
    int msg_motor_id;

    //actuator
    pr2_hardware_interface::Actuator* actuator;

    //Data we can read from the motor
    int strain_gauge_left;
    int strain_gauge_right;

    int flags;

    double temperature;

    int can_msgs_received;
    int can_msgs_transmitted;

    int firmware_svn_revision;

    int tests;

    int force_control_p;
    int force_control_i;
    int force_control_d;
    int force_control_imax;
    int force_control_deadband;

    /**
     * this boolean is set to true as long as we receive the
     * data from the motor.
     */
    bool motor_ok;
    /**
     * this boolean is set to true if the data coming from the motor
     * through the CAN bus are messed up.
     */
    bool bad_data;
  };

  struct Joint
  {
    std::string joint_name;

    //the indexes of the joints in the joint array
    // coming from the hardware which are used to
    // compute the joint data.
    JointToSensor joint_to_sensor;

    bool has_motor;
    boost::shared_ptr<Motor> motor;
  };

  typedef threadsafe::Map<boost::shared_ptr<shadow_robot::JointCalibration> > CalibrationMap;
}

namespace shadow_robot
{
  class SrRobotLib
  {
  public:
    SrRobotLib(std::vector<std::string> joint_names, std::vector<int> motor_ids,
               std::vector<shadow_joints::JointToSensor> joint_to_sensors,
               std::vector<pr2_hardware_interface::Actuator*> actuators,
               shadow_joints::CalibrationMap calibration_map) {};
    ~SrRobotLib() {};

    void update(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data);

    boost::ptr_vector<shadow_joints::Joint> joints_vector;
    shadow_joints::CalibrationMap calibration_map;

    int main_pic_idle_time;
    int main_pic_idle_time_min;

  protected:
    virtual void initialize_maps(std::vector<std::string> joint_names, std::vector<int> motor_ids,
                                 std::vector<shadow_joints::JointToSensor> joint_to_sensors,
                                 std::vector<pr2_hardware_interface::Actuator*> actuators) = 0;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif

