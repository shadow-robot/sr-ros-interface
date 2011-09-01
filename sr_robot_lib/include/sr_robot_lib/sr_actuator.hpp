/**
 * @file   sr_actuator.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Jun 28 16:19:41 2011
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
 * @brief This redefines the actuator to incorporate the information coming
 *  from our motor boards.
 *
 *
 */

#ifndef _SR_ACTUATOR_HPP_
#define _SR_ACTUATOR_HPP_

#include <pr2_hardware_interface/hardware_interface.h>

namespace sr_actuator
{
  class SrActuatorState
  {
  public:
    SrActuatorState() :
      timestamp_(0),
      device_id_(0),
      position_(0),
      velocity_(0),
      is_enabled_(0),
      halted_(0),
      last_commanded_current_(0),
      last_executed_current_(0),
      last_measured_current_(0),
      last_commanded_effort_(0),
      last_executed_effort_(0),
      last_measured_effort_(0),
      motor_voltage_(0),
      strain_gauge_left_(0),
      strain_gauge_right_(0),
      temperature_(0),
      can_msgs_received_(0),
      can_msgs_transmitted_(0),
      pic_firmware_svn_revision_(0),
      server_firmware_svn_revision_(0),
      firmware_modified_(0),
      serial_number_low(0),
      serial_number_high(0),
      motor_gear_ratio(0),
      assembly_data_year(0),
      assembly_data_month(0),
      assembly_data_day(0),
      tests_(0),
      can_error_counters(0),
      force_control_f_(0),
      force_control_p_(0),
      force_control_i_(0),
      force_control_d_(0),
      force_control_imax_(0),
      force_control_deadband_(0),
      force_control_sign_(0),
      force_control_frequency_(0)
    {}

    /**
     * The time at which actuator state was measured, relative to the time the ethercat process was started.
     * Timestamp value is not synchronised with wall time and may be different for different actuators.
     * For Willow Garage motor controllers, timestamp is made when actuator data is sampled.
     * sample_timestamp_ will provide better accuracy than ros::Time::now() or robot->getTime()
     * when using a time difference in calculations based on actuator variables.
     */
    ros::Duration sample_timestamp_;

    /** The time at which this actuator state was measured (in seconds).
     * This value should be same as sample_timestamp_.toSec() for Willow Garage devices.
     * The timestamp_ variable is being kept around for backwards compatibility, new controllers
     * should use sample_timestamp_ instead.
     */
    double timestamp_;

    int device_id_; //!< Position in EtherCAT chain

    double position_; //!< The position of the motor (in radians)
    double velocity_; //!< The velocity in radians per second

    bool is_enabled_; //!< Enable status
    bool halted_; //!< indicates if the motor is halted. A motor can be halted because of voltage or communication problems

    double last_commanded_current_; //!< The current computed based on the effort specified in the ActuatorCommand (in amps)
    double last_executed_current_; //!< The actual current requested after safety limits were enforced (in amps)
    double last_measured_current_; //!< The measured current (in amps)

    double last_commanded_effort_; //!< The torque requested in the previous ActuatorCommand (in Nm)
    double last_executed_effort_; //!< The torque applied after safety limits were enforced (in Nm)
    double last_measured_effort_; //!< The measured torque (in Nm)

    double max_effort_; //!< Absolute torque limit for actuator (derived from motor current limit). (in Nm)

    double motor_voltage_; //!< Motor voltage (in volts)

    unsigned long long int  strain_gauge_left_;
    unsigned long long int strain_gauge_right_;

    std::vector<int> raw_sensor_values_;
    std::vector<double> calibrated_sensor_values_;

    /**
     * a vector containing human readable flags:
     *  each flag is a pair containing the flag name
     *  and a boolean which is true if the flag is
     *  really serious, false if it's just a warning
     *  flag.
     */
    std::vector<std::pair<std::string, bool> > flags_;

    double temperature_;

    unsigned int can_msgs_received_;
    unsigned int can_msgs_transmitted_;

    unsigned int pic_firmware_svn_revision_;
    unsigned int server_firmware_svn_revision_;
    bool firmware_modified_;

    unsigned int serial_number_low;
    unsigned int serial_number_high;

    unsigned int motor_gear_ratio;
    unsigned int assembly_data_year;
    unsigned int assembly_data_month;
    unsigned int assembly_data_day;

    int tests_;
    unsigned int can_error_counters;

    int force_control_f_;
    int force_control_p_;
    int force_control_i_;
    int force_control_d_;
    int force_control_imax_;
    int force_control_deadband_;
    int force_control_sign_;
    int force_control_frequency_;

    int force_control_pterm;
    int force_control_iterm;
    int force_control_dterm;
  }; //end class SrActuatorState



  class SrActuator : public pr2_hardware_interface::Actuator
  {
  public:
    SrActuator()
      : pr2_hardware_interface::Actuator()
    {};

    SrActuator(std::string name)
      : pr2_hardware_interface::Actuator(name)
    {};

    SrActuatorState state_;
  }; //end class SrActuator
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif

