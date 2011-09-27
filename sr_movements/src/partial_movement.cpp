/**
 * @file   partial_movement.cpp
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

#include "sr_movements/partial_movement.hpp"

namespace shadowrobot
{
  PartialMovement::PartialMovement()
  {}

  PartialMovement::~PartialMovement()
  {}

  double PartialMovement::get_target(double percentage)
  {
    if( percentage < 0.0)
      percentage = 0.0;
    if( percentage > 1.0)
      percentage = 1.0;

    int index = static_cast<int>( steps.size() * static_cast<double>(percentage) );
    return steps[index];
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

