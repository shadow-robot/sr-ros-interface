/**
 * @file   calibration.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Jun 10 12:06:45 2011
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
 * @brief This class contains the algorithm which takes the raw ADC reading,
 *        and uses the N-point piecewise linear algorithm to convert it to
 *        an angle in degrees.
 *
 *
 */

#include "sr_robot_lib/calibration.hpp"

#include <boost/assert.hpp>
#include <algorithm>

#include <boost/foreach.hpp>
#include <iostream>

namespace shadow_robot
{
  JointCalibration::JointCalibration(std::vector<joint_calibration::Point> calibration_table)
  {
    this->calibration_table_ = calibration_table;
    //calibration size - 1 because we use this number to access the last value in the vector
    calibration_table_size_ = calibration_table.size() - 1;

    //fails if we have only one calibration point as it's not possible
    //to interpolate from one point
    BOOST_ASSERT(calibration_table_size_ > 0);

    /*
     * make sure that the given calibration table is ordered by
     * growing values of the raw_value
     */
    std::sort(this->calibration_table_.begin(), this->calibration_table_.end(), joint_calibration::sort_growing_raw_operator);
  }

  JointCalibration::~JointCalibration()
  {}

  /**
   * Computes the calibrated joint position from the ADC raw reading.
   *
   * @param raw_reading the reading from the ADC
   *
   * @return the calibrated joint position in radians.
   */
  double JointCalibration::compute(double raw_reading)
  {
    /**
     * the two points from the calibration which we'll use
     * to do the linear interpolation.
     */
    joint_calibration::Point low_point, high_point;

    //That takes care of computing a reading that's before
    // the calibration table as well as a reading that's in the
    // first calibration table case.
    low_point  = calibration_table_[0];
    high_point = calibration_table_[1];

    bool found = false;

    //if we have more than 2 points in our calibration table
    // or if the raw value isn't before the calibration table
    if( (raw_reading > calibration_table_[0].raw_value) )
    {
      if( (calibration_table_size_ > 1) )
      {
        for(unsigned int index_cal=1; index_cal < calibration_table_size_; ++index_cal)
        {
          if( (raw_reading >= calibration_table_[index_cal - 1].raw_value) &&
              (raw_reading < calibration_table_[index_cal].raw_value) )
          {
            low_point  = calibration_table_[index_cal - 1];
            high_point = calibration_table_[index_cal];

            found = true;
            break;
          }
        } //end for

        //the point is outside of the table
        if( !found )
        {
          low_point = calibration_table_[calibration_table_size_ - 1];
          high_point = calibration_table_[calibration_table_size_];
        }
      } // end if 2 values only in the table
    } //end if raw_reading before table

    return linear_interpolate_(raw_reading, low_point, high_point);
  }

  /**
   * Interpolate linearly between the 2 points, for the given raw_value
   *
   * y = y0 + (x-x0)*((y1-y0)/(x1-x0))
   *
   * @param raw_reading the X value (raw_reading) to compute the interpolation for.
   * @param low_point the first point of our line
   * @param high_point the second point of our line
   *
   * @return the computed Y value (calibrated value)
   */
  double JointCalibration::linear_interpolate_(double raw_reading,
                                               joint_calibration::Point low_point,
                                               joint_calibration::Point high_point)
  {
    //y1 - y0
    double y = high_point.calibrated_value - low_point.calibrated_value;
    // (y1 - y0) / (x1 - x0)
    y /= (high_point.raw_value - low_point.raw_value);
    //  (x-x0)*((y1-y0)/(x1-x0))
    y *= (raw_reading - low_point.raw_value);
    //  y0 + (x-x0)*((y1-y0)/(x1-x0))
    y += low_point.calibrated_value;

    return y;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


