/**
 * @file   sr_self_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Feb 4, 2013
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
 * @brief Class containing the self tests for the Shadow Robot EtherCAT hardware.
 *
 *
 */

#include "sr_self_test/sr_self_test.hpp"
#include <sr_utilities/sr_math_utils.hpp>
#include <boost/filesystem.hpp>

namespace shadow_robot
{
  const double SrSelfTest::MAX_MSE_CONST_ = 0.18;

  TestJointMovement::TestJointMovement(std::string joint_name, std::pair<double, double> min_max)
    : mse(0.0), nh_tilde_("~")
  {
    joint_name_ = joint_name;

    //subscribes to the mean square error (published by movement pub below)
    mse_sub_ = nh_tilde_.subscribe("mse_out", 1, &TestJointMovement::mse_cb_, this);

    //initialises the movement publisher
    std::string img_path;
    nh_tilde_.getParam("image_path", img_path);

    mvt_from_img_.reset(new shadowrobot::MovementFromImage(img_path) );

    double min, max, publish_rate;
    unsigned int repetition, nb_mvt_step;
    min = min_max.first;
    max = min_max.second;
    publish_rate = 10.0;
    repetition = 1;
    nb_mvt_step = 1000;
    std::string controller_type = "sr";

    mvt_pub_.reset(new shadowrobot::MovementPublisher(min, max, publish_rate, repetition,
                                                      nb_mvt_step, controller_type));
    mvt_pub_->add_movement( *mvt_from_img_.get() );

    sub_ = nh_tilde_.subscribe("/sh_"+joint_name+"_mixed_position_velocity_controller/state", nb_mvt_step, &shadowrobot::MovementPublisher::calculateErrorCallback, mvt_pub_.get());
    sub_state_ = nh_tilde_.subscribe("/sh_"+joint_name+"_mixed_position_velocity_controller/state", nb_mvt_step, &TestJointMovement::state_cb_, this);

    pub_ = nh_tilde_.advertise<std_msgs::Float64>("/sh_"+joint_name+"_mixed_position_velocity_controller/command", 5);

    mvt_pub_->set_subscriber(sub_);
    mvt_pub_->set_publisher(pub_);
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

  SrSelfTest::SrSelfTest(bool simulated)
    : nh_tilde_("~")
  {
    simulated_ = simulated;

    //rename existing folder if it exists
    if( boost::filesystem::exists("/tmp/self_tests") )
    {
      //delete the last backup if it exists
      if( boost::filesystem::exists("/tmp/self_tests.bk") )
        boost::filesystem::remove_all("/tmp/self_tests.bk");

      //backup last test plots
      boost::filesystem::rename("/tmp/self_tests", "/tmp/self_tests.bk");
    }
    //create folder in /tmp for storing the plots
    path_to_plots_ = "/tmp/self_tests/"+ros::this_node::getName();
    boost::filesystem::create_directories(path_to_plots_);

    test_runner_.setID("12345");

    //add the different tests
    ROS_ERROR("1");
    test_services_();
    ROS_ERROR("2");
    add_all_movements_tests_();
    ROS_ERROR("ready");
  }

  void SrSelfTest::add_all_movements_tests_()
  {
    std::pair<double, double> min_max;

    //TODO: min max and joint names should be read from urdf
    min_max.first = 0.0;
    min_max.second = sr_math_utils::to_rad(180.0);
    joints_to_test_.push_back("ffj0");
    min_and_maxs_.push_back(min_max);

    min_max.second = sr_math_utils::to_rad(90.0);
    joints_to_test_.push_back("ffj3");
    min_and_maxs_.push_back(min_max);

    index_joints_to_test_ = 0;

    for(size_t i=0; i < joints_to_test_.size(); ++i)
    {
      ROS_ERROR_STREAM(" adding " << i << "/" << joints_to_test_.size() );
      //test_runner_.add("Check movements", this, &SrSelfTest::test_movement_);
      ROS_ERROR("toto");
    }
  }

  void SrSelfTest::test_movement_(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    std::string joint_name = joints_to_test_[index_joints_to_test_];
    std::pair<double, double> min_max = min_and_maxs_[index_joints_to_test_];


    std::string img_path;
    if( !nh_tilde_.getParam("image_path", img_path) )
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Parameter image_path not set, can't analyse movements.");
      return;
    }

    test_mvts_[joint_name].reset( new TestJointMovement(joint_name, min_max) );

    //wait a bit for mse to be received
    ros::Duration(1.0).sleep();

    //plot the data
    test_runner_.plot(test_mvts_[joint_name]->values, path_to_plots_+joint_name+".png");

    //check they are correct
    std::stringstream diag_msg;
    diag_msg << "Movement for " << joint_name << " (mse = " << test_mvts_[joint_name]->mse <<")";
    if(test_mvts_[joint_name]->mse < MAX_MSE_CONST_)
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, diag_msg.str());
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, diag_msg.str());

    if( index_joints_to_test_ + 1 < joints_to_test_.size() )
      ++index_joints_to_test_;
  }

  void SrSelfTest::test_services_()
  {
    std::vector<std::string> services_to_test;
    services_to_test.push_back("/pr2_controller_manager/list_controller_types");
    services_to_test.push_back("/pr2_controller_manager/list_controllers");
    services_to_test.push_back("/pr2_controller_manager/load_controller");
    services_to_test.push_back("/pr2_controller_manager/reload_controller_libraries");
    services_to_test.push_back("/pr2_controller_manager/switch_controller");
    services_to_test.push_back("/pr2_controller_manager/unload_controller");

    test_runner_.addServicesTest(services_to_test);
  }
}  // namespace shadow_robot


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


