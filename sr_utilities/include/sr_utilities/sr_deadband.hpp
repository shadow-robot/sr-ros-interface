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
#include <deque>

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
     * Hysteresis deadband: we average the last N errors. If this average is less
     * than a small deadband, we "enter the deadband zone" and send only a command
     * of zero to the motor. We leave the deadband zone if the average of the error
     * is getting bigger than x*deadband, or if we receive a new command.
     */
    HysteresisDeadband() :
      last_demand(static_cast<T>(0.0)), entered_small_deadband(false)
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
     * @param deadband_multiplicator the value by which we multiply the deadband
     *                               to have the bigger deadband against which we
     *                               check for leaving the deadband zone
     * @param nb_errors_for_avg the nb of errors we keep for averaging
     *
     * @return true if the demand is in the deadband.
     */
    bool is_in_deadband(T demand, T error, T deadband, double deadband_multiplicator = 5.0, unsigned int nb_errors_for_avg = 50)
    {
      bool is_in_deadband = false;

      last_errors.push_back( error );
      double avg_error = 0.0;
      for( unsigned int i = 0 ; i < last_errors.size(); ++i )
      {
        avg_error += last_errors[i];
      }
      avg_error /= last_errors.size();

      // Received a new command:
      if( last_demand != demand )
      {
        entered_small_deadband = false;
        last_demand = demand;
      }
      else
      {
        //check if we entered the small deadband
        if( !entered_small_deadband )
        {
          entered_small_deadband = fabs(avg_error) < deadband;
        }

        //we always compute the error if we haven't entered the small deadband
        if (!entered_small_deadband)
          is_in_deadband = false;
        else
        {
          if( fabs(avg_error) > deadband_multiplicator*deadband ) //we're outside of the big deadband -> compute the error
          {
            is_in_deadband = false;
            //when we leave the big deadband we wait until we're back in the small deadband before stopping the motor
            entered_small_deadband = false;
          }
          else                                 //we're in the big deadband -> send a force demand of 0.0
            is_in_deadband = true;
        }
      }

      if( last_errors.size() > nb_errors_for_avg )
        last_errors.pop_front();

      return is_in_deadband;
    };

  private:
    T deadband;
    T last_demand;
    std::deque<T> last_errors;
    bool entered_small_deadband;
  };

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
