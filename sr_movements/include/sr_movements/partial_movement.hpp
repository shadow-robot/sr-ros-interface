/**
 * @file   partial_movement.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep 27 10:05:01 2011
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
 * @brief  This is the main class from which all the different types of movement
 *         will inherit.
 *
 */


#ifndef _PARTIAL_MOVEMENT_HPP_
#define _PARTIAL_MOVEMENT_HPP_

#include <vector>

namespace shadowrobot
{
  class PartialMovement
  {
  public:
    PartialMovement();
    virtual ~PartialMovement();

    /**
     * Returns a target for the given percentage.
     *
     * @param percentage must be between 0 and 1
     *
     * @return target from this movement.
     */
    double get_target(double percentage);

  protected:
    /**
     * A vector containing the steps for this movement, in percentage
     * of the range. Contains -1 if no target for this point.
     */
    std::vector<double> steps;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
