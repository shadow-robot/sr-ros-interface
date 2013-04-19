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
    path_to_plots_ = "/tmp/self_tests/"+ros::this_node::getName() + "/";
    boost::filesystem::create_directories(path_to_plots_);

    test_runner_.setID("12345");

    //add the different tests
    test_services_();

    //some tests can only be run on the real hand
    if(!simulated)
    {
      //add manual tests (tactile, calibration)
      test_runner_.addManualTests();
      //parses the diagnostics to find common problems
      test_runner_.add_diagnostic_parser();
      //test the noise of the sensors
      test_runner_.addSensorNoiseTest();
    }

    //calling this from a oneshot timer because we're using the
    // hand commander which needs the hand to be fully initialised
    // before we can instantiate it.
    // Using 30s sleep here is a pain... not sure how to best get
    // rid of that... testing for availability of some services may
    // be not enough.
    test_movement_timer_ = nh_tilde_.createTimer( ros::Duration(30.0),
                                                  &SrSelfTest::add_all_movements_tests_, this,
                                                  true );
  }

  void SrSelfTest::test_services_()
  {
    std::vector<std::string> services_to_test;
    services_to_test.push_back("pr2_controller_manager/list_controller_types");
    services_to_test.push_back("pr2_controller_manager/list_controllers");
    services_to_test.push_back("pr2_controller_manager/load_controller");
    services_to_test.push_back("pr2_controller_manager/reload_controller_libraries");
    services_to_test.push_back("pr2_controller_manager/switch_controller");
    services_to_test.push_back("pr2_controller_manager/unload_controller");

    test_runner_.addServicesTest(services_to_test);
  }

  ///////
  // TESTING MOVEMENTS

  void SrSelfTest::add_all_movements_tests_(const ros::TimerEvent& event)
  {
    if( hand_commander_ == NULL )
      hand_commander_.reset(new shadowrobot::HandCommander());

    joints_to_test_ = hand_commander_->get_all_joints();

    //send all the joints to their safe positions:
    for( size_t i=0; i<joints_to_test_.size(); ++i)
    {
      send_safe_target_(joints_to_test_[i]);
      ros::Duration(0.4).sleep();
    }
    index_joints_to_test_ = 0;
    for(size_t i=0; i < joints_to_test_.size(); ++i)
    {
      test_runner_.add("Check movements", this, &SrSelfTest::test_movement_);
    }
  }

  void SrSelfTest::test_movement_(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    std::string joint_name = joints_to_test_[index_joints_to_test_];

    //sends the joint to a collision safe position
    send_safe_target_(joint_name);
    ros::Duration(0.5).sleep();

    std::string img_path;
    if( !nh_tilde_.getParam("image_path", img_path) )
    {
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Parameter image_path not set, can't analyse movements.");
      return;
    }

    test_mvts_[joint_name].reset( new TestJointMovement(joint_name) );

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
    else
      index_joints_to_test_ = 0;
  }

  void SrSelfTest::send_safe_target_(std::string joint_name)
  {
    if( safe_targets_ == NULL )
      init_safe_targets_();

    update_safe_targets_(joint_name);

    std::vector<sr_robot_msgs::joint> joint_vector;
    joint_vector.resize( joints_to_test_.size() );

    for( size_t i=0; i<joints_to_test_.size(); ++i)
    {
      std::map<std::string, sr_robot_msgs::joint>::iterator safe_target;
      safe_target = safe_targets_->find( joints_to_test_[i] );
      if( safe_target == safe_targets_->end() )
      {
        //joint not found in the map -> use the min
        joint_vector[i].joint_name = joints_to_test_[i];
        joint_vector[i].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max( joints_to_test_[i] ).first );
      }
      else
      {
        joint_vector[i].joint_name = joints_to_test_[i];
        joint_vector[i].joint_target = safe_target->second.joint_target;
      }
    }

    hand_commander_->sendCommands(joint_vector);
    ros::spinOnce();
  }

  void SrSelfTest::update_safe_targets_(std::string joint_name)
  {
    //This is very hugly.... not sure how to make it prettier easily - haven't much time...
    if( joint_name.compare("FFJ4") == 0 )
    {
      (*safe_targets_.get())["FFJ4"].joint_target = 0.0;
      (*safe_targets_.get())["MFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("MFJ4").second );
      (*safe_targets_.get())["RFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("RFJ4").first );
      (*safe_targets_.get())["LFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("LFJ4").first );
    }
    else
    {
      if( joint_name.compare("MFJ4") == 0 )
      {
        (*safe_targets_.get())["FFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("FFJ4").first );
        (*safe_targets_.get())["MFJ4"].joint_target = 0.0;
        (*safe_targets_.get())["RFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("RFJ4").first );
        (*safe_targets_.get())["LFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("LFJ4").first );
      }
      else
      {
        if( joint_name.compare("RFJ4") == 0 )
        {
          (*safe_targets_.get())["FFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("FFJ4").first );
          (*safe_targets_.get())["MFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("MFJ4").first );
          (*safe_targets_.get())["RFJ4"].joint_target = 0.0;
          (*safe_targets_.get())["LFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("LFJ4").first );
        }
        else
        {
          if( joint_name.compare("LFJ4") == 0 )
          {
            (*safe_targets_.get())["FFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("FFJ4").first );
            (*safe_targets_.get())["MFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("MFJ4").first );
            (*safe_targets_.get())["RFJ4"].joint_target = sr_math_utils::to_degrees( hand_commander_->get_min_max("RFJ4").second );
            (*safe_targets_.get())["LFJ4"].joint_target = 0.0;
          }
          else
          {
            (*safe_targets_.get())["FFJ4"].joint_target = 0.0;
            (*safe_targets_.get())["MFJ4"].joint_target = 0.0;
            (*safe_targets_.get())["RFJ4"].joint_target = 0.0;
            (*safe_targets_.get())["LFJ4"].joint_target = 0.0;
          }
        }
      }
    }
    //we ignore the other joints
  }

  void SrSelfTest::init_safe_targets_()
  {
    safe_targets_.reset( new std::map<std::string, sr_robot_msgs::joint>());
    sr_robot_msgs::joint safe_target;

    //??J4 -> min or max or 0... need to find a way to do that
    safe_target.joint_name = "FFJ4";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );
    safe_target.joint_name = "MFJ4";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );
    safe_target.joint_name = "RFJ4";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );
    safe_target.joint_name = "LFJ4";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );

    //wrist -> 0
    safe_target.joint_name = "WRJ1";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );
    safe_target.joint_name = "WRJ2";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );

    //thumb -> 0
    safe_target.joint_name = "THJ2";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );
    safe_target.joint_name = "THJ3";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );
    safe_target.joint_name = "THJ4";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );
    safe_target.joint_name = "THJ5";
    safe_target.joint_target = 0.0;
    safe_targets_->insert( std::pair<std::string, sr_robot_msgs::joint>(safe_target.joint_name, safe_target) );
  }

}  // namespace shadow_robot


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


