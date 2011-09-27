/**
 * @file   sr_movements.hpp
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
 * @brief  Contains a node which can be used to take the hand through a series
 *         of movements (perfect for tuning controllers for example).
 *
 */


#ifndef _SR_MOVEMENTS_HPP_
#define _SR_MOVEMENTS_HPP_

#include <ros/ros.h>

#include <boost/thread.hpp>

namespace shadowrobot
{
  class SrMovements
  {
  public:
    SrMovements();
    virtual ~SrMovements();
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
