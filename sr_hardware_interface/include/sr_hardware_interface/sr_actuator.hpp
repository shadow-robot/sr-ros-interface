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

#include "sr_hardware_interface/tactile_sensors.hpp"
#include <ros_ethercat_model/hardware_interface.hpp>

namespace sr_actuator
{

class SrActuatorState
{
public:
  SrActuatorState() :
    position_unfiltered_(0.0),
    can_msgs_received_(0),
    can_msgs_transmitted_(0),
    pic_firmware_svn_revision_(0),
    server_firmware_svn_revision_(0),
    firmware_modified_(0),
    serial_number_low_set(false),
    serial_number_high_set(false),
    serial_number_low(0),
    serial_number_high(0),
    serial_number(0),
    assembly_data_year(0),
    assembly_data_month(0),
    assembly_data_day(0),
    tests_(0),
    can_error_counters(0)
  {
  }

  //Serial number is composed of a low and a high byte.
  void set_serial_number_low(unsigned int serial)
  {
    serial_number_low = serial;
    serial_number_low_set = true;

    if (serial_number_high_set) //we received both bytes
      compute_serial();
  };
  void set_serial_number_high(unsigned int serial)
  {
    serial_number_high = serial;
    serial_number_high_set = true;

    if (serial_number_low_set) //we received both bytes
      compute_serial();
  };
  void compute_serial()
  {
    serial_number = serial_number_low + (serial_number_high * 65536);
  };

  double position_unfiltered_;

  uint64_t can_msgs_received_;
  uint64_t can_msgs_transmitted_;

  unsigned int pic_firmware_svn_revision_;
  unsigned int server_firmware_svn_revision_;
  bool firmware_modified_;

  bool serial_number_low_set;
  bool serial_number_high_set;
  unsigned int serial_number_low;
  unsigned int serial_number_high;
  unsigned int serial_number;

  unsigned int assembly_data_year;
  unsigned int assembly_data_month;
  unsigned int assembly_data_day;

  int tests_;
  unsigned int can_error_counters;

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

  std::vector<tactiles::AllTactileData>* tactiles_;
}; //end class SrActuatorState

class SrMotorActuatorState : public SrActuatorState
{
public:
  SrMotorActuatorState() :
    strain_gauge_left_(0),
    strain_gauge_right_(0),
    force_unfiltered_(0.0),
    pwm_(0),
    motor_gear_ratio(0),
    force_control_f_(0),
    force_control_p_(0),
    force_control_i_(0),
    force_control_d_(0),
    force_control_imax_(0),
    force_control_deadband_(0),
    force_control_sign_(0),
    force_control_frequency_(0),
    temperature_(0.0)
  {
  }

  signed short strain_gauge_left_;
  signed short strain_gauge_right_;

  double force_unfiltered_;

  int pwm_;

  unsigned int motor_gear_ratio;

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
  double temperature_;
}; //end class SrMotorActuatorState

class SrMuscleActuatorState : public SrActuatorState
{
public:
  SrMuscleActuatorState() :
    pressure_(),
    last_commanded_valve_()
  {
  }

  ///Pressure sensors for each of the two muscles of the actuator
  uint16_t pressure_[2];
  int8_t last_commanded_valve_[2];
}; //end class SrMuscleActuatorState

class SrMuscleActuatorCommand
{
public:
  SrMuscleActuatorCommand() :
    valve_()
  {
  }

  int8_t valve_[2];
}; //end class SrMuscleActuatorCommand

/**
 * This class defines a Motor actuator
 */
class SrMotorActuator : public ros_ethercat_model::Actuator
{
public:

  SrMotorActuatorState motor_state_;
}; //end class SrMotorActuator

/**
 * This class defines a Muscle actuator
 */
class SrMuscleActuator : public ros_ethercat_model::Actuator
{
public:

  SrMuscleActuatorState muscle_state_;
  SrMuscleActuatorCommand muscle_command_;
}; //end class SrMotorActuator
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */

#endif

