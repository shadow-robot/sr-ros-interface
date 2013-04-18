/**
 * @file   test_joint_movement.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Mar 27 05:47:00 2013
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
 * @brief  Class used for testing the controllers of a given joint.
 *
 *
 */

#include "sr_self_test/test_joint_movement.hpp"

namespace shadow_robot
{
  TestJointMovement::TestJointMovement(std::string joint_name)
    : mse(0.0), nh_tilde_("~")
  {
    joint_name_ = joint_name;

    //subscribes to the mean square error (published by movement pub below)
    mse_sub_ = nh_tilde_.subscribe("mse_out", 1, &TestJointMovement::mse_cb_, this);

    //initialises the movement publisher
    std::string img_path;
    nh_tilde_.getParam("image_path", img_path);

    mvt_from_img_.reset(new shadowrobot::MovementFromImage(img_path) );

    double publish_rate;
    unsigned int repetition, nb_mvt_step;
    publish_rate = 10.0;
    repetition = 1;
    nb_mvt_step = 1000;
    std::string controller_type = "sr";

    mvt_pub_.reset(new shadowrobot::MovementPublisher(joint_name, publish_rate, repetition,
                                                      nb_mvt_step, controller_type));
    mvt_pub_->add_movement( *mvt_from_img_.get() );

    sub_state_ = nh_tilde_.subscribe( mvt_pub_->get_subscriber_topic(), nb_mvt_step,
                                      &TestJointMovement::state_cb_, this );

    mvt_pub_->start();
  }

  void TestJointMovement::state_cb_(const sr_robot_msgs::JointControllerState::ConstPtr& msg)
  {
    values[joint_name_ +" position"].push_back(msg->process_value);
    values[joint_name_ +" target"].push_back(msg->set_point);
    values[joint_name_ +" error"].push_back(msg->error);
  }

  void TestJointMovement::mse_cb_(const std_msgs::Float64::ConstPtr& msg)
  {
    mse = msg->data;

    //unsubscribe after receiving the message
  }
}
