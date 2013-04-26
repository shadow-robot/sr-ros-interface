/**
 * @file   motor_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Apr 22 05:47:43 2013
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

#include <boost/algorithm/string.hpp>
#include "sr_self_test/motor_test.hpp"
#include <pr2_mechanism_msgs/ListControllers.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <std_msgs/Float64.h>

namespace shadow_robot
{
  MotorTest::MotorTest(self_test::TestRunner* test_runner,
                       std::string joint_name,
                       shadowrobot::HandCommander* hand_commander)
    : test_runner_(test_runner), joint_name_( joint_name ), hand_commander_(hand_commander),
      record_data_(0)
  {
    //joint name is lower case in the controller topics
    boost::algorithm::to_lower(joint_name_);

    test_runner_->add("Test motor ["+joint_name_+"]", this, &MotorTest::run_test);

    //subscribe to diagnostic topic
    diagnostic_sub_ = nh_.subscribe("diagnostics_agg", 1, &MotorTest::diagnostics_agg_cb_, this);
  }

  void MotorTest::run_test(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    std::string current_ctrl, effort_ctrl;
    //list currently running controllers and find the one for effort control
    pr2_mechanism_msgs::ListControllers list_ctrl;
    if( ros::service::call("pr2_controller_manager/list_controllers", list_ctrl))
    {
      for (size_t i = 0; i < list_ctrl.response.controllers.size(); ++i)
      {
        if( list_ctrl.response.controllers[i].find( joint_name_ ) != std::string::npos )
        {
          if( list_ctrl.response.state[i].compare( "running" ) == 0 )
          {
            current_ctrl = list_ctrl.response.controllers[i];
          }
          if( list_ctrl.response.controllers[i].find( "_effort_controller" ) != std::string::npos )
          {
            effort_ctrl = list_ctrl.response.controllers[i];
          }
        }
      }
    }
    else
    {
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Failed to list controllers - aborting test." );
    }

    //load the effort controller if necessary
    if( effort_ctrl.compare("") == 0 )
    {
      effort_ctrl = "sh_"+joint_name_+"_effort_controller";
      pr2_mechanism_msgs::LoadController load_ctrl;
      load_ctrl.request.name = effort_ctrl;
      if( !ros::service::call("pr2_controller_manager/load_controller", load_ctrl))
      {
        status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                        "Failed to load controller "+load_ctrl.request.name+" - aborting test." );
      }
    }

    //switching to effort controllers (in PWM -> send direct PWM demand)
    pr2_mechanism_msgs::SwitchController switch_ctrl;
    switch_ctrl.request.start_controllers.push_back(effort_ctrl);
    switch_ctrl.request.stop_controllers.push_back(current_ctrl);
    switch_ctrl.request.strictness = pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
    if( !ros::service::call("pr2_controller_manager/switch_controller", switch_ctrl))
    {
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Failed to switch from controller "+current_ctrl+" to controller "+effort_ctrl +" - aborting test." );
    }

    //apply effort and start recording data
    effort_pub_ = nh_.advertise<std_msgs::Float64>(effort_ctrl+"/command", 5, true);
    std_msgs::Float64 target;
    ros::Rate rate(2.0);

    //first one way
    target.data = 150.0;
    record_data_ = 1;

    for (unsigned int i=0; i < 50; ++i)
    {
      effort_pub_.publish(target);
      rate.sleep();
    }
    // @todo: analyse data

    //then the other
    target.data = -150.0;
    record_data_ = -1;
    for (unsigned int i=0; i < 50; ++i)
    {
      effort_pub_.publish(target);
      rate.sleep();
    }

    //reset to previous control mode
    record_data_ = 0;
    switch_ctrl.request.start_controllers.push_back(current_ctrl);
    switch_ctrl.request.stop_controllers.push_back(effort_ctrl);
    switch_ctrl.request.strictness = pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
    if( !ros::service::call("pr2_controller_manager/switch_controller", switch_ctrl))
    {
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Failed to switch from controller "+effort_ctrl+" to controller "+current_ctrl +" - aborting test." );
    }

    //test succeeded
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Test passed.");
  }

  void MotorTest::diagnostics_agg_cb_(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
  {
    //ignore the data if we're not recording.
    if(record_data_ == 0)
      return;

    if(record_data_ == 1)
    {
      ROS_ERROR("Recording +");
    }
    else if (record_data_ == -1)
    {
      ROS_ERROR("Recording -");
    }
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

