/**
 * @file   test_self_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Feb  7 09:57:59 2013
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
 * @brief  Testing the self test library using gtest
 *
 *
 */

#include "sr_self_test/sr_self_test.hpp"
#include <ros/ros.h>

#include <diagnostic_msgs/SelfTest.h>


class MyNode
{
public:

  // self_test::TestRunner is the handles sequencing driver self-tests.
  shadow_robot::SrTestRunner self_test_;

  // A value showing statefulness of tests
  double some_val;

  ros::NodeHandle nh_;

  MyNode() : self_test_()
  {
    self_test_.setID("12345");

    std::vector<std::string> services_to_test;
    services_to_test.push_back("/sr_self_test_test/self_test");
    services_to_test.push_back("/rosout/get_loggers");

    self_test_.addServicesTest(services_to_test);
  }

  bool spin()
  {
    while (nh_.ok())
    {
      ros::Duration(1).sleep();

      self_test_.checkTest();
    }
    return true;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr_self_test_test");

  MyNode n;
  n.spin();

  return(0);
}
