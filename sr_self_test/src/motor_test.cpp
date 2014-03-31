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
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <sr_robot_msgs/ChangeControlType.h>
#include <std_msgs/Float64.h>
#include <sstream>

namespace shadow_robot
{
  const double MotorTest::STANDARD_PWM_TARGET_ = 150.0; //Used for most of the motors
  const double MotorTest::WRJ1_PWM_TARGET_ = 250.0; //Used for WRJ1
  const double MotorTest::WRJ2_PWM_TARGET_ = 190.0; //Used for WRJ2

  const int MotorTest::STRAIN_GAUGE_THRESHOLD_ = 40; //the minimum value of the SG when pulling in that direction

  MotorTest::MotorTest(self_test::TestRunner* test_runner,
                       std::string joint_name,
                       shadowrobot::HandCommander* hand_commander)
    : test_runner_(test_runner), joint_name_( joint_name ), hand_commander_(hand_commander),
      record_data_(0), test_current_zero_(true), test_current_moving_(true), test_strain_gauge_right_(true), test_strain_gauge_left_(true), PWM_target_(STANDARD_PWM_TARGET_)
  {
    //joint name is lower case in the controller topics
    boost::algorithm::to_lower(joint_name_);

    if(joint_name_.compare("wrj1") == 0)
      PWM_target_ = WRJ1_PWM_TARGET_;
    else if(joint_name_.compare("wrj2") == 0)
      PWM_target_ = WRJ2_PWM_TARGET_;
    else 
      PWM_target_ = STANDARD_PWM_TARGET_;

    test_runner_->add("Test motor ["+joint_name_+"]", this, &MotorTest::run_test);
  }

  void MotorTest::run_test(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    //reset test results
    test_current_zero_ = true;
    test_current_moving_ = true;
    test_strain_gauge_right_ = true;
    test_strain_gauge_left_ = true;

    //check current control type
    sr_robot_msgs::ControlType current_ctrl_type;
    sr_robot_msgs::ChangeControlType change_ctrl_type;
    change_ctrl_type.request.control_type.control_type = sr_robot_msgs::ControlType::QUERY;
    if( ros::service::call("realtime_loop/change_control_type", change_ctrl_type) )
    {
      current_ctrl_type = change_ctrl_type.response.result;
    }
    else
    {
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Failed to get current control type - aborting test." );
      return;
    }

    //switch to PWM
    change_ctrl_type.request.control_type.control_type = sr_robot_msgs::ControlType::PWM;
    if( !ros::service::call("realtime_loop/change_control_type", change_ctrl_type) )
    {
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Failed to change to PWM control type - aborting test." );
      return;
    }

    //subscribe to diagnostic topic
    diagnostic_sub_ = nh_.subscribe("diagnostics_agg", 1, &MotorTest::diagnostics_agg_cb_, this);

    std::string current_ctrl, effort_ctrl;
    //list currently running controllers and find the one for effort control
    controller_manager_msgs::ListControllers list_ctrl;
    if( ros::service::call("controller_manager/list_controllers", list_ctrl))
    {
      for (size_t i = 0; i < list_ctrl.response.controller.size(); ++i)
      {
        if( list_ctrl.response.controller[i].name.find( joint_name_ ) != std::string::npos )
        {
          if( list_ctrl.response.controller[i].state.compare( "running" ) == 0 )
          {
            current_ctrl = list_ctrl.response.controller[i].name;
          }
          if( list_ctrl.response.controller[i].name.find( "_effort_controller" ) != std::string::npos )
          {
            effort_ctrl = list_ctrl.response.controller[i].name;
          }
        }
      }
    }
    else
    {
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Failed to list controllers - aborting test." );
      return;
    }

    //load the effort controller if necessary
    if( effort_ctrl.compare("") == 0 )
    {
      effort_ctrl = "sh_"+joint_name_+"_effort_controller";
      controller_manager_msgs::LoadController load_ctrl;
      load_ctrl.request.name = effort_ctrl;
      if( !ros::service::call("controller_manager/load_controller", load_ctrl))
      {
        status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                        "Failed to load controller "+load_ctrl.request.name+" - aborting test." );
      }
    }

    //switching to effort controllers (in PWM -> send direct PWM demand)
    controller_manager_msgs::SwitchController switch_ctrl;
    switch_ctrl.request.start_controllers.push_back(effort_ctrl);
    switch_ctrl.request.stop_controllers.push_back(current_ctrl);
    switch_ctrl.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    if( !ros::service::call("controller_manager/switch_controller", switch_ctrl))
    {
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Failed to switch from controller "+current_ctrl+" to controller "+effort_ctrl +" - aborting test." );
      return;
    }

    //apply effort and start recording data
    effort_pub_ = nh_.advertise<std_msgs::Float64>(effort_ctrl+"/command", 5, true);
    std_msgs::Float64 target;
    ros::Rate rate(2.0);

    //first one way
    target.data = PWM_target_;
    record_data_ = 1;

    for (unsigned int i=0; i < 10; ++i)
    {
      effort_pub_.publish(target);
      rate.sleep();
    }

    //then the other
    target.data = -PWM_target_;
    record_data_ = -1;
    for (unsigned int i=0; i < 10; ++i)
    {
      effort_pub_.publish(target);
      rate.sleep();
    }

    //then nothing (to check that current is back to 0)
    target.data = 0.0;
    record_data_ = 0;
    for (unsigned int i=0; i < 10; ++i)
    {
      effort_pub_.publish(target);
      rate.sleep();
    }

    //stop the subscriber
    diagnostic_sub_.shutdown();

    //reset to previous control mode
    change_ctrl_type.request.control_type = current_ctrl_type;
    if( !ros::service::call("realtime_loop/change_control_type", change_ctrl_type) )
    {
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Failed to change back to the previous control type - aborting test." );
      return;
    }

    switch_ctrl.request.start_controllers.push_back(current_ctrl);
    switch_ctrl.request.stop_controllers.push_back(effort_ctrl);
    switch_ctrl.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT;
    if( !ros::service::call("controller_manager/switch_controller", switch_ctrl))
    {
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Failed to switch from controller "+effort_ctrl+" to controller "+current_ctrl +" - aborting test." );
      return;
    }

    //test results
    std::stringstream ss;
    if( test_current_zero_ && test_current_moving_ &&
        test_strain_gauge_right_ && test_strain_gauge_left_ )
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Test passed.");
      return;
    }

    //there was an error during the test
    else
    {
      ss << "Test failed: ";
      if(!test_current_zero_)
        ss << " Current was not zero when the motor was stopped. ";
      if(!test_current_moving_)
        ss << " Current was out of specified interval when the motor was driven. ";
      if(!test_strain_gauge_right_)
        ss << " Problem with Strain Gauge Right.";
      if(!test_strain_gauge_left_)
        ss << " Problem with Strain Gauge Left.";
    }

    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, ss.str());
  }

  void MotorTest::diagnostics_agg_cb_(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
  {
    for( size_t status_i = 0; status_i < msg->status.size(); ++status_i )
    {
      if( msg->status[status_i].name.find("SRDMotor "+boost::algorithm::to_upper_copy(joint_name_) ) != std::string::npos )
      {
        for (size_t value_i = 0; value_i < msg->status[status_i].values.size(); ++value_i)
        {
          if( msg->status[status_i].values[value_i].key.compare("Measured Current") == 0 )
          {
            double current = ::atof( msg->status[status_i].values[value_i].value.c_str() );

            //not sending any targets, current should be close to 0
            if( record_data_ == 0)
            {
              if( current > 0.016 )
              {
                test_current_zero_ = false;
                ROS_DEBUG("Current zero test error: %f", current);
              }
              else
              {
                test_current_zero_ = true;
              }
            }
            else
            {
              //sending some targets, current should be between ? and ?
              //@todo find min max values for current when driving motors
              if( current > 0.5 || current < 0.012 )
              {
                test_current_moving_ = false;
                ROS_DEBUG("Current moving test error: %f", current);
              }
              else
              {
                test_current_moving_ = true;
              }
            }
          }
          else if( msg->status[status_i].values[value_i].key.compare("Strain Gauge Left") == 0 &&
                   record_data_ == -1 )
          {
            //@todo is it always SGL for + and SGR for - ??
            int sgl = ::atoi( msg->status[status_i].values[value_i].value.c_str() );
            //@todo check min value for SG under tension
            if( sgl < STRAIN_GAUGE_THRESHOLD_ )
            {
              test_strain_gauge_left_ = false;
              ROS_DEBUG("sgl test error: %d", sgl);
            }
            else
            {
              test_strain_gauge_left_ = true;
            }
          }
          else if( msg->status[status_i].values[value_i].key.compare("Strain Gauge Right") == 0 &&
                   record_data_ == 1)
          {
            //@todo same here
            int sgr = ::atoi( msg->status[status_i].values[value_i].value.c_str() );
            if( sgr < STRAIN_GAUGE_THRESHOLD_ )
            {
              test_strain_gauge_right_ = false;
              ROS_DEBUG("sgr test error: %d", sgr);
            }
            else
            {
              test_strain_gauge_right_ = true;
            }
          }
        }
      }

    }
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

