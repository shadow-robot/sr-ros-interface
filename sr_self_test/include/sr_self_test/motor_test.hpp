/**
 * @file   motor_test.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Apr 22 05:43:10 2013
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
 *
 * @brief Run different tests on the motors (apply PWM offset
 *        and measure current, test strain gauges, etc.. )
 *
 *
 */

#ifndef _MOTOR_TEST_H_
#define _MOTOR_TEST_H_

#include <self_test/self_test.h>

namespace shadow_robot
{
  class MotorTest
  {
  public:
    MotorTest(self_test::TestRunner* test_runner);
    virtual ~MotorTest() {};

    void run_test(diagnostic_updater::DiagnosticStatusWrapper& status);

  protected:
    self_test::TestRunner* test_runner_;
  };
}

  /* For the emacs weenies in the crowd.
     Local Variables:
     c-basic-offset: 2
     End:
  */

#endif /* _MOTOR_TEST_H_ */
