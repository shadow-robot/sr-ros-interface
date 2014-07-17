/**
 * @file   calibration.hpp
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

#ifndef _SR_CALIBRATION_HPP_
#define _SR_CALIBRATION_HPP_

#include <iostream>
#include <vector>

namespace joint_calibration
{
  /**
   * A point in the N-point piecewise
   * linear calibration
   */
  struct Point
  {
    double raw_value;
    double calibrated_value;
  };

  /**
   * This function is used by the sort algorithm to compare
   * 2 different points and sort the calibration table by
   * growing values of raw_value.
   *
   * @param p1 the first point to compare
   * @param p2 the second point to compare
   *
   * @return true if p1.raw_value < p2.raw_value
   */
  static bool sort_growing_raw_operator(const Point& p1, const Point& p2)
  {
    return p1.raw_value < p2.raw_value;
  }
}

namespace shadow_robot
{
  /**
   * This class is used to compute the calibrated joint position, given a raw
   * ADC sensor reading, using a N-point piecewise linear calibration table.
   */
  class JointCalibration
  {
  public:
    JointCalibration(std::vector<joint_calibration::Point> calibration_table);

    /**
     * Computes the calibrated joint position from the ADC raw reading.
     *
     * @param raw_reading the reading from the ADC
     *
     * @return the calibrated joint position in radians.
     */
    double compute(double raw_reading);

    /**
     * Overload the << operator, for easier debugging.
     */
    friend std::ostream& operator<<(std::ostream& out, const JointCalibration& calib )
    {
      out << " calibration = {";
      out << "size: " << calib.calibration_table_size_;
      for( unsigned int i=0; i< calib.calibration_table_.size(); ++i)
      {
        out << " [raw: " << calib.calibration_table_[i].raw_value;
        out << ", cal: " << calib.calibration_table_[i].calibrated_value<<"]";
      }
      out << " }";
      return out;
    };

  private:
    /**
     * The calibration table. The vector is ordered by growing values
     * of the raw_data
     */
    std::vector<joint_calibration::Point> calibration_table_;
    /**
     * The size of the calibration table, used for quick access
     */
    unsigned int calibration_table_size_;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif

