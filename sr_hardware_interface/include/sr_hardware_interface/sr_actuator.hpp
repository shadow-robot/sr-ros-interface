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
  class SrActuatorState : public pr2_hardware_interface::ActuatorState
  {
  public:
    SrActuatorState() :
      pr2_hardware_interface::ActuatorState(),
      strain_gauge_left_(0),
      strain_gauge_right_(0),
      temperature_(0),
      pwm_(0),
      can_msgs_received_(0),
      can_msgs_transmitted_(0),
      pic_firmware_svn_revision_(0),
      server_firmware_svn_revision_(0),
      firmware_modified_(0),
      serial_number_low(0),
      serial_number_high(0),
      serial_number(0),
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

    int pwm_;

    unsigned int can_msgs_received_;
    unsigned int can_msgs_transmitted_;

    unsigned int pic_firmware_svn_revision_;
    unsigned int server_firmware_svn_revision_;
    bool firmware_modified_;

    //Serial number is composed of a low and
    // a high byte.
    void set_serial_number_low(unsigned int serial)
    {
      serial_number_low = serial;

      if( serial_number_high != 0 ) //we received both bytes
        compute_serial();
    };
    void set_serial_number_high(unsigned int serial)
    {
      serial_number_high = serial;

      if( serial_number_low != 0 ) //we received both bytes
        compute_serial();
    };
    void compute_serial()
    {
      serial_number = serial_number_low + (serial_number_high * 65536);
    };
    unsigned int serial_number_low;
    unsigned int serial_number_high;
    unsigned int serial_number;

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

