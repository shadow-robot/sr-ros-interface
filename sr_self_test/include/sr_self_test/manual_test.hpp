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
 * @brief
 *
 *
 */

#ifndef _MANUAL_TEST_H_
#define _MANUAL_TEST_H_

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <sr_robot_msgs/BiotacAll.h>
#include <sr_robot_msgs/ShadowPST.h>

namespace shadow_robot
{
  class ManualTests
  {
  public:
    ManualTests();
    virtual ~ManualTests()
    {};

    void run_manual_tests(diagnostic_updater::DiagnosticStatusWrapper& status);

  private:
    ros::NodeHandle nh_;

    ///ROS subscriber to the PS3 tactile topic
    ros::Subscriber tactile_ps3_sub_;
    void tactile_ps3_cb_(sr_robot_msgs::BiotacAllConstPtr& msg);
    ///ROS subscriber to the PST tactile topic
    ros::Subscriber tactile_pst_sub_;
    void tactile_pst_cb_(sr_robot_msgs::ShadowPSTConstPtr& msg);
  };
} //end namespace

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _MANUAL_TEST_H_ */
