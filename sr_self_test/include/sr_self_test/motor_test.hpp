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
#include <sr_hand/hand_commander.hpp>

namespace shadow_robot
{
  class MotorTest
  {
  public:
    MotorTest( self_test::TestRunner* test_runner,
               std::string joint_name,
               shadowrobot::HandCommander* hand_commander);
    virtual ~MotorTest() {};

    void run_test(diagnostic_updater::DiagnosticStatusWrapper& status);

  protected:
    ros::NodeHandle nh_;
    self_test::TestRunner* test_runner_;
    std::string joint_name_;
    shadowrobot::HandCommander* hand_commander_;
    ros::Publisher effort_pub_;
    ros::Subscriber diagnostic_sub_;
    double PWM_target_;
    ///0 if not recording, 1 if going +, -1 if going -
    short record_data_;

    bool test_current_zero_;
    bool test_current_moving_;
    bool test_strain_gauge_right_;
    bool test_strain_gauge_left_;

    static const double STANDARD_PWM_TARGET_;
    static const double WRJ1_PWM_TARGET_;
    static const double WRJ2_PWM_TARGET_;
    static const int STRAIN_GAUGE_THRESHOLD_;

    /**
     * Susbscribed to the diagnostics_agg topic.
     * @param msg new incoming msg
     */
    void diagnostics_agg_cb_(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
  };
}

  /* For the emacs weenies in the crowd.
     Local Variables:
     c-basic-offset: 2
     End:
  */

#endif /* _MOTOR_TEST_H_ */
