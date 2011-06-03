/**
 * @file   sr_robot_lib.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Fri Jun  3 12:12:13 2011
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
 * @brief This is a generic robot library for Shadow Robot's Hardware.
 * You can find it instantiated in the sr_edc_ethercat_drivers.
 *
 *
 */

#ifndef _SR_ROBOT_LIB_HPP_
#define _SR_ROBOT_LIB_HPP_

namespace shadow_robot
{
  class SrRobotLib
  {
  pubic:
    SrRobotLib();
    ~SrRobotLib();
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif

