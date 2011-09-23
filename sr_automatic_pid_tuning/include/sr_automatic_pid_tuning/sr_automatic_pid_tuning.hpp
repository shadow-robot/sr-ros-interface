/**
 * @file   sr_automatic_pid_tuning.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Sep 22 11:36:52 2011
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
 * @brief  Automatic pid tuning using a genetic algorithm, based on the evodev
 *         library.
 *
 *
 */

#ifndef _SR_AUTOMATIC_PID_TUNING_HPP_
#define _SR_AUTOMATIC_PID_TUNING_HPP_

#include <es/make_real.h>
#include <apply.h>

#include <boost/smart_ptr.hpp>

namespace shadow_robot
{
  class SrAutomaticPidTuning
  {
  public:
    SrAutomaticPidTuning(eoParser parser);
    virtual ~SrAutomaticPidTuning();

    void run();
  protected:
    eoState state;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
