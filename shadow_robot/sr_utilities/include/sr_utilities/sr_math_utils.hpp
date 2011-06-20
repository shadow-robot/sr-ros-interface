/**
 * @file   sr_math_utils.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Mon Jun 20 10:20:27 2011
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
 * @brief This is a header library used to implement some useful math
 * functions. It is used in our different packages.
 *
 *
 */

#ifndef _SR_MATH_UTILS_HPP_
#define _SR_MATH_UTILS_HPP_

namespace sr_math_utils
{
  static inline int ipow(int base, int exp)
  {
    int result = 1;
    while (exp)
    {
      if (exp & 1)
        result *= base;
      exp >>= 1;
      base *= base;
    }

    return result;
  }

  static inline bool is_bit_mask_index_true(int bit_mask, int index)
  {
    if ( bit_mask & (1<<index) )
      return true;
    else
      return false;
  }

  static inline bool is_bit_mask_index_false(int bit_mask, int index)
  {
    return !(is_bit_mask_index_true(bit_mask, index));
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
