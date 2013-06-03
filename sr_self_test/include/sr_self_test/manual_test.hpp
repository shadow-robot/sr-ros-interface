/**
 * @file   manual_test.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Apr 16 06:03:09 2013
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
 * @brief We want to be able to run some tests with the user input (ask the
 *        user to choose whether the test failed or passed). This can be used
 *        for example for visually checking the calibration or pressing the
 *        tactiles manually.
 *
 *
 */

#ifndef _MANUAL_TEST_H_
#define _MANUAL_TEST_H_

#include <diagnostic_updater/DiagnosticStatusWrapper.h>

namespace shadow_robot
{
  class ManualTests
  {
  public:
    ManualTests( std::string message, int id);
    virtual ~ManualTests()
    {};

    void run_manual_tests(diagnostic_updater::DiagnosticStatusWrapper& status);

  private:
    ros::NodeHandle nh_;

    ///the message we want the user to see
    std::string message_;
    ///an id for the test
    int id_;

    ///Service client for getting the user input
    ros::ServiceClient user_input_client_;
  };
} //end namespace

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _MANUAL_TEST_H_ */
