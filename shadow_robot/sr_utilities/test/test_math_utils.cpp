/**
 * @file   test_math_utils.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Jun 20 10:27:36 2011
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
 * @brief  Testing the math utils library.
 *
 *
 */

#include "sr_utilities/sr_math_utils.hpp"
#include <gtest/gtest.h>

TEST(Pow, base2)
{
  EXPECT_EQ(sr_math_utils::ipow(2, 0), 1);
  EXPECT_EQ(sr_math_utils::ipow(2, 2), 4);
  EXPECT_EQ(sr_math_utils::ipow(2, 10), 1024);
}

TEST(Pow, base3)
{
  EXPECT_EQ(sr_math_utils::ipow(3, 0), 1);
  EXPECT_EQ(sr_math_utils::ipow(3, 2), 9);
  EXPECT_EQ(sr_math_utils::ipow(3, 10), 59049);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
