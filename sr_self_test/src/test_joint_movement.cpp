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
#include <stdio.h>
#include "boost/iostreams/device/file_descriptor.hpp"
#include "boost/iostreams/stream.hpp"


namespace shadow_robot
{
  TestJointMovement::TestJointMovement(std::string joint_name, shadowrobot::HandCommander* hand_commander)
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
    publish_rate = 100.0;
    repetition = 1;
    nb_mvt_step = 10000;

    if( hand_commander != NULL )
      hand_commander_.reset(hand_commander);
    else
      hand_commander_.reset(new shadowrobot::HandCommander());

    std::string controller_state_topic = hand_commander_->get_controller_state_topic(joint_name);
    std::string controller_state_topic_type = get_ROS_topic_type(controller_state_topic);
    std::string controller_type = "";
    if(controller_state_topic_type.compare("sr_robot_msgs/JointControllerState") == 0)
    {
      controller_type = "sr";
    }

    mvt_pub_.reset(new shadowrobot::MovementPublisher(joint_name, publish_rate, repetition,
                                                      nb_mvt_step, controller_type, false,
                                                      hand_commander ));
    mvt_pub_->add_movement( *mvt_from_img_.get() );

    if(controller_type.compare("sr") == 0)
    {
      sr_sub_state_ = nh_tilde_.subscribe( mvt_pub_->get_subscriber_topic(), nb_mvt_step,
                                           &TestJointMovement::sr_state_cb_, this );
    }
    else
    {
      sub_state_ = nh_tilde_.subscribe( mvt_pub_->get_subscriber_topic(), nb_mvt_step,
                                        &TestJointMovement::state_cb_, this );
    }

    mvt_pub_->start();
  }

  void TestJointMovement::sr_state_cb_(const sr_robot_msgs::JointControllerState::ConstPtr& msg)
  {
    values[joint_name_ +" position"].push_back(msg->process_value);
    values[joint_name_ +" target"].push_back(msg->set_point);
    values[joint_name_ +" error"].push_back(msg->error);
  }

  void TestJointMovement::state_cb_(const control_msgs::JointControllerState::ConstPtr& msg)
  {
    values[joint_name_ +" position"].push_back(msg->process_value);
    values[joint_name_ +" target"].push_back(msg->set_point);
    values[joint_name_ +" error"].push_back(msg->error);
  }

  void TestJointMovement::mse_cb_(const std_msgs::Float64::ConstPtr& msg)
  {
    mse = msg->data;

    //unsubscribe after receiving the message
    mse_sub_.shutdown();
    sub_state_.shutdown();
  }

  std::string TestJointMovement::get_ROS_topic_type(std::string topic_name)
  {
    typedef boost::iostreams::file_descriptor_source boost_fd;
    typedef boost::iostreams::stream<boost_fd> boost_stream; 
    
    FILE *myfile;
    std::string cmd;

    cmd = "rostopic type " + topic_name;

    // make sure to popen and it succeeds
    if(!(myfile = popen(cmd.c_str(), "r")))
    {
      ROS_ERROR_STREAM("Command failed: " << cmd);
    }

    boost_fd fd(fileno(myfile), boost::iostreams::never_close_handle);
    boost_stream stream(fd);
    //stream.set_auto_close(false); // https://svn.boost.org/trac/boost/ticket/3517
    std::string topic_type;
    if( !std::getline(stream,topic_type))
    {
      ROS_ERROR_STREAM("Could nod read line from get_ROS_topic_type command");
    }

    pclose(myfile);
    return topic_type;
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

