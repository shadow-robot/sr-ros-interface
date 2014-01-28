/**
 * @file   real_shadowhand.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue May 25 17:51:10 2010
 *
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
 * @brief The real shadowhand is the ROS interface to Shadow Robot robotic hand.
 *
 * @todo Make sure it works with the motor hand.
 *
 */

#ifndef   	REAL_SHADOWHAND_H_
# define   	REAL_SHADOWHAND_H_

#include "sr_hand/hand/sr_articulated_robot.h"

namespace sr_self_tests
{
  ///The number of targets to send during the test.
  static const unsigned int nb_targets_to_send = 100;

  ///the size of the msgs_frequency array
  static const unsigned int msgs_frequency_size = 5;
  //the rate at which we'll publish the data
  static const int msgs_frequency[msgs_frequency_size] = {1, 5, 10, 20, 100};

  ///the size of the joints_to_test array
  static const unsigned int joints_to_test_size = 1;
  ///the name of the joint on which we want to run the test.
  static const std::string joints_to_test[joints_to_test_size] = {"FFJ3"};
}

namespace shadowrobot
{
/**
 * The real shadowhand class is a class used to access the C code of the Dextrous Hand.
 */
  class RealShadowhand : public SRArticulatedRobot
  {
  public:
    /**
     * Constructor for the real Shadowhand: initialize the connection to the robot and also initialize the
     * joint_map containing the mapping between joint_names and their data.
     */
    RealShadowhand();

    virtual ~RealShadowhand();

    /**
     * Send a new target to a given joint on the robot. The command will be issued to the robot which will move if
     * the controllers are started and tuned. The target is truncated to the correct range.
     *
     * @param joint_name The Joint in joints_map you wish to send the target to.
     * @param target The target in degree
     * @return 0 if success ; -1 if error
     */
    virtual short sendupdate( std::string joint_name, double target );

    virtual JointData getJointData( std::string joint_name );
    virtual JointsMap getAllJointsData();

    virtual short setContrl( std::string contrlr_name, JointControllerData ctrlr_data );
    virtual JointControllerData getContrl( std::string contrlr_name );
    virtual short setConfig( std::vector<std::string> myConfig );
    virtual void getConfig( std::string joint_name );
    virtual std::vector<DiagnosticData> getDiagnostics();
  protected:
    /***
     * Initializes the mapping between the joint_names and their data. This function fetches the joint_names from
     * the robot code.
     */
    void initializeMap();

      /////////////////
     //    TESTS    //
    /////////////////

    /**
     * A set of tasks to run before the tests:
     *  - Ensure we don't use the ROS interface during the tests (lock
     *    the mutexes).
     *
     *
     * @param status the status of the pretest
     */
    void pretest(diagnostic_updater::DiagnosticStatusWrapper& status);

    /**
     * A set of tasks to run after the tests:
     *  - release the mutexes
     *
     *
     * @param status the status of the posttest
     */
    void posttest(diagnostic_updater::DiagnosticStatusWrapper& status);

    /**
     * A test to check the number of messages received is the same as the
     * number of messages sent. Calls the test_messages_routine for each
     * joint / frequency we're testing.
     *
     * @param status the test result.
     */
    void test_messages(diagnostic_updater::DiagnosticStatusWrapper& status);

    /**
     * The routine called during the test: test the number of received messages
     * for a given joint, a given number of times and at a given frequency.
     *
     * @param joint_name the name of the joint to test
     * @param repeat the number of messages to send
     * @param rate the rate at which we send the messages
     *
     * @return a pair containing the status of the test (error or ok), and a message
     *         to display.
     */
    std::pair<unsigned char, std::string> test_messages_routine(std::string joint_name, unsigned int repeat, ros::Rate rate);

  }; //end class
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif 	    /* !REAL_SHADOWHAND_H_ */
