/*
 * File:  test_hand_finder.cpp
 * Author: Vahid Aminzadeh <vahid@shadowrobot.com>
 * Copyright:
 *
 * @brief see README.md
 */
#include <gtest/gtest.h>
#include <vector>
#include "ros/ros.h"
#include <ros/package.h>
#include <iostream>
#include <map>
#include <string>
#include "sr_utilities/sr_hand_finder.hpp"
using std::vector;
using std::map;
using std::string;

TEST(SrHandFinder, hand_absent_test)
{
  shadow_robot::SrHandFinder hand_finder;
  map<string, vector<string> > hand_joints(hand_finder.get_joints());
  ASSERT_EQ(hand_joints.size(), 0);
  map<string, string> calibration_path(hand_finder.get_calibration_path());
  ASSERT_EQ(calibration_path.size(), 0);
  shadow_robot::HandControllerTuning controller_tuning(hand_finder.get_hand_controller_tuning());
  ASSERT_EQ(controller_tuning.friction_compensation_.size(), 0);
  ASSERT_EQ(controller_tuning.host_control_.size(), 0);
  ASSERT_EQ(controller_tuning.motor_control_.size(), 0);
}
TEST(SrHandFinder, hand_present_test)
{
  ros::NodeHandle nh;
  nh.setParam("hand/mapping/1", "rh");
  nh.setParam("hand/joint_prefix/1", "rh_");
  const string joint_names[] = {"FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
                                "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
                                "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"};
  shadow_robot::SrHandFinder hand_finder;
  map<string, vector<string> > hand_joints(hand_finder.get_joints());
  for (map<string, vector<string> >::const_iterator iter = hand_joints.begin(); iter != hand_joints.end(); ++iter)
  {
    for (size_t i = 0; i != iter->second.size(); ++i)
    {
      ROS_DEBUG_STREAM(iter->second[i]);
      ASSERT_STREQ(iter->second[i].c_str(), ("rh_" +  joint_names[i]).c_str());
    }
  }
  string ethercat_path = ros::package::getPath("sr_ethercat_hand_config");
  map<string, string> calibration_path(hand_finder.get_calibration_path());
  for (map<string, string>::const_iterator iter = calibration_path.begin(); iter != calibration_path.end(); ++iter)
  {
    ROS_INFO_STREAM(iter->second);
    ASSERT_STREQ(iter->second.c_str(), (ethercat_path + "/calibrations/rh/" + "calibration.yaml").c_str());
  }
  shadow_robot::HandControllerTuning controller_tuning(hand_finder.get_hand_controller_tuning());
  for (map<string, string>::const_iterator iter = controller_tuning.friction_compensation_.begin();
      iter != controller_tuning.friction_compensation_.end(); ++iter)
  {
    ASSERT_STREQ(iter->second.c_str(), (ethercat_path  + "/controls/friction_compensation.yaml").c_str());
    ROS_DEBUG_STREAM(iter->second);
  }
  for (map<string, string>::const_iterator iter = controller_tuning.motor_control_.begin();
        iter != controller_tuning.motor_control_.end(); ++iter)
  {
    ASSERT_STREQ(iter->second.c_str(),
                 (ethercat_path  + "/controls/motors/rh/motor_board_effort_controllers.yaml").c_str());
    ROS_DEBUG_STREAM(iter->second);
  }
  for (map<string, vector<string> >::const_iterator iter = controller_tuning.host_control_.begin();
          iter != controller_tuning.host_control_.end(); ++iter)
  {
    string host_array[] = {"sr_edc_calibration_controllers.yaml",
                         "sr_edc_joint_velocity_controllers_PWM.yaml",
                         "sr_edc_effort_controllers_PWM.yaml",
                         "sr_edc_joint_velocity_controllers.yaml",
                         "sr_edc_effort_controllers.yaml",
                         "sr_edc_mixed_position_velocity_joint_controllers_PWM.yaml",
                         "sr_edc_joint_position_controllers_PWM.yaml",
                         "sr_edc_mixed_position_velocity_joint_controllers.yaml",
                         "sr_edc_joint_position_controllers.yaml"};
    vector<string> host_controller_files(host_array, host_array + 9);
    ASSERT_EQ(host_controller_files.size(), iter->second.size());
    for (size_t i = 0; i != iter->second.size(); ++i)
    {
      ASSERT_STREQ(iter->second[i].c_str(), (ethercat_path + "/controls/host/rh/" + host_controller_files[i]).c_str());
      ROS_DEBUG_STREAM(iter->second[i]);
    }
  }
  nh.deleteParam("hand/mapping/1");
  nh.deleteParam("hand/joint_prefix/1");
  ASSERT_TRUE(true);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_hand_finder");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

