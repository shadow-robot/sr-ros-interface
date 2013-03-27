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
#include <sr_robot_msgs/joint.h>
#include <sr_hand/hand_commander.hpp>
#include <ros/ros.h>

#include "sr_self_test/test_joint_movement.hpp"

namespace shadow_robot
{
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
     * Sends a "safe target" to all the joints: we want to avoid the collisions
     *  when running the movement tests.
     *
     * @param joint_name the joint we're going to move.
     */
    void send_safe_target_(std::string joint_name);
    ///A map storing the safe targets for the joints (only those different than min)
    boost::shared_ptr<std::map<std::string, sr_robot_msgs::joint> > safe_targets_;
    /**
     * Initialises the map safe_targets_
     */
    void init_safe_targets_();
    /**
     * Updates the map safe_targets_ based on the joint we're going to move.
     *  For example ??J4 safe values are different depending on which joint
     *   4 has been moved already.
     *
     * @param joint_name The name of the joint we're going to move.
     */
    void update_safe_targets_(std::string joint_name);

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
