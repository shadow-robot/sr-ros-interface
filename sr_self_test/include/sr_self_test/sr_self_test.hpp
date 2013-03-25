/**
 * @file   sr_self_test.hpp
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

#ifndef SR_SELF_TEST_HPP_
#define SR_SELF_TEST_HPP_

#include <diagnostic_msgs/SelfTest.h>

#include "sr_self_test/sr_test_runner.hpp"

#include <boost/thread.hpp>
#include <sr_movements/movement_from_image.hpp>
#include <sr_movements/movement_publisher.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sr_robot_msgs/JointControllerState.h>
#include <sr_hand/hand_commander.hpp>

namespace shadow_robot
{
  class TestJointMovement
  {
  public:
    TestJointMovement(std::string joint_name);
    ~TestJointMovement() {};

    double mse;
    std::map<std::string, std::vector<double> > values;

  private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    ros::Subscriber sub_state_;
    void state_cb_(const sr_robot_msgs::JointControllerState::ConstPtr& msg);

    ros::Subscriber mse_sub_;
    void mse_cb_(const std_msgs::Float64::ConstPtr& msg);

    boost::shared_ptr<shadowrobot::MovementFromImage> mvt_from_img_;
    boost::shared_ptr<shadowrobot::MovementPublisher> mvt_pub_;

    ros::NodeHandle nh_tilde_;

    boost::shared_ptr<boost::thread> thread_;

    std::string joint_name_;
  };

  class SrSelfTest {
  public:
    SrSelfTest(bool simulated);
    ~SrSelfTest() {};

    void checkTest()
    {
      test_runner_.checkTest();
    }

  private:
    // self_test::TestRunner is the handles sequencing driver self-tests.
    shadow_robot::SrTestRunner test_runner_;

    ///The hand commander is used for getting a list of all controlled joints
    boost::shared_ptr<shadowrobot::HandCommander> hand_commander_;

    bool simulated_;

    void test_services_();

    std::vector<std::string> joints_to_test_;

    size_t index_joints_to_test_;
    void add_all_movements_tests_(const ros::TimerEvent& event);
    void test_movement_(diagnostic_updater::DiagnosticStatusWrapper& status);
    ros::Timer test_movement_timer_;

    /**
     * Sends a "safe target" to the given joint: we want to avoid the collisions
     *  when running the movement tests.
     *
     * @param joint_name the joint we want to put in a safe position.
     */
    void send_safe_target_(std::string joint_name);
    ///A map storing the safe targets for the joints (only those different than min)
    boost::shared_ptr<std::map<std::string, sr_robot_msgs::joint> > safe_targets_;
    /**
     * Initialises the map safe_targets_
     */
    void init_safe_targets_();

    ros::NodeHandle nh_tilde_;

    std::map<std::string, boost::shared_ptr<TestJointMovement> > test_mvts_;

    static const double MAX_MSE_CONST_;

    std::string path_to_plots_;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* SR_SELF_TEST_HPP_ */
