/**
 * @file   sr_deadband.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Aug 18 09:58:24 2011
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
 * @brief This library contains different types of deadbands, which are
 * useful in controllers for example.
 *
 */

#ifndef _SR_DEADBAND_HPP_
#define _SR_DEADBAND_HPP_

#include <sr_utilities/sr_math_utils.hpp>

namespace sr_deadband
{
  /**
   * A simple deadband : returns true if the demand is
   * in the deadband.
   *
   * @param value is this value in the deadband?
   * @param deadband the width of the deadband (the deadband is symetric)
   *
   * @return true if the demand is in the deadband
   */

  template <class T>
  static inline bool simple_deadband (T value, T deadband)
  {
    return ( fabs(value) > deadband );
  }

  template <class T>
  class HysteresisDeadband
  {
  public:
    /**
     * Hysteresis deadband: each time the error is changing sign, we check if the
     * error is leaving a deadband which is larger than the noise. (i.e. go to the
     * target, then once you're there, don't send any motor demand unless the
     * position error is more than the sensor noise.
     *
     */
    HysteresisDeadband() :
      last_demand(static_cast<T>(0.0)), last_error(static_cast<T>(0.0)), changed_sign_since_new_command(false)
    {
    };

    ~HysteresisDeadband()
    {};

    /**
     * Are we in the hysteresis deadband? If we are in it, then we're just
     * sending a force demand of 0.
     *
     * @param demand The demand
     * @param error The error (demand - actual value)
     * @param deadband the deadband value
     *
     * @return true if the demand is in hte deadband.
     */
    bool is_in_deadband(T demand, T error, T deadband)
    {
      bool is_in_deadband;

      // Received a new command:
      if( last_demand != demand )
      {
        changed_sign_since_new_command = false;
        last_error = error;
      }
      //check if the error changed sign since we received a new command
      if( !changed_sign_since_new_command )
        changed_sign_since_new_command = ( sr_math_utils::sign(error) != sr_math_utils::sign(last_error) );

      //we always compute the error if we still haven't changed sign
      if (!changed_sign_since_new_command)
        is_in_deadband = false;
      else
      {
        if( fabs(error) > deadband ) //we're outside of the deadband -> compute the error
          is_in_deadband = false;
        else                                 //we're in the deadband -> send a force demand of 0.0
          is_in_deadband = true;
      }
      //save the last error in position and the last command
      last_error = error;
      last_demand = demand;

      return is_in_deadband;
    };

  private:
    T deadband;
    T last_demand, last_error;
    bool changed_sign_since_new_command;
  };

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
