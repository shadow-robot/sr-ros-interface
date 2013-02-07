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

#include <gtest/gtest.h>

class SrSelfTestTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    nh_ = ros::NodeHandle("~");
    test_client_ = nh_.serviceClient<diagnostic_msgs::SelfTest>("self_test");
  }

  ros::NodeHandle nh_;
  ros::ServiceClient test_client_;
  shadow_robot::SrSelfTest self_test_;
};

TEST_F(SrSelfTestTest, constructor)
{
  EXPECT_TRUE(true);
  ROS_ERROR("segfault on destructor? oO");
}

TEST_F(SrSelfTestTest, call_test_service)
{
  ROS_ERROR("this is not called");

  EXPECT_TRUE(test_client_.waitForExistence(ros::Duration(10.0)));

  diagnostic_msgs::SelfTest srv;
  if(test_client_.call(srv))
    EXPECT_TRUE(true);
  else
    EXPECT_TRUE(false);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "sr_self_test_test");
  // init the node

  return RUN_ALL_TESTS();
}
