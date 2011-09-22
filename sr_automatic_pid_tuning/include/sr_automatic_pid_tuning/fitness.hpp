/**
 * @file   fitness.hpp
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
 * @brief  Computes the fitness for the given genotype.
 *
 *
 */

#ifndef _SR_AUTOMATIC_PID_TUNING_HPP_
#define _SR_AUTOMATIC_PID_TUNING_HPP_

#include <vector>
//-----------------------------------------------------------------------------


/** Just a simple function that takes an eoEsBase<double> and sets the fitnes
    to sphere
    @param _ind  vector<double>
*/

double real_value(const std::vector<double>& _ind)
{
  double sum = 0;
  for (unsigned i = 0; i < _ind.size(); i++)
      sum += _ind[i] * _ind[i];
  return sqrt(sum);
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
