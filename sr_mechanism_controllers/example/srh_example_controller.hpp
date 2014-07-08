/**
 * @file   srh_example_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Nov  7 10:31:05 2011
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
 * @brief This is a simple controller example, showing you how to create your
 *        own controller, how to receive the data and update the command.
 *
 */


#ifndef _SRH_EXAMPLE_CONTROLLER_HPP_
#define _SRH_EXAMPLE_CONTROLLER_HPP_

#include <sr_mechanism_controllers/sr_controller.hpp>

namespace controller
{
  class SrhExampleController : public SrController
  {
  public:
    /**
     * The controller manager will instanciate one instance of this
     * class per controlled joint.
     */
    SrhExampleController();

    /**
     * Destructor.
     */
    virtual ~SrhExampleController();

    /**
     * The first init is used to read which joint is being controlled
     * from the parameter server.
     *
     * @param robot A pointer to the robot state, passed to the 2nd init function.
     * @param n The ROS nodehandle, to be able to access the parameter server.
     *
     * @return True if the 2nd init function succeeds.
     */
    bool init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n);

    /**
     * This init funciton is called by the previous init function. It
     * finishes to initalise the different necessary variables.
     *
     * @param robot A pointer to the robot state (used to get the joint_state)
     * @param joint_name The name of the joint which is controlled.
     *
     * @return true if initialized.
     */
    bool init( ros_ethercat_model::RobotState *robot, const std::string &joint_name);

    /**
     * This method is called when the controller is started. The command is then
     * to be the current position (or effort / velocity / ... depending on what
     * you're controlling), so that the first command won't move the joint.
     *
     */
    virtual void starting(const ros::Time& time);

    /**
     * Issues commands to the joint. This method is called at the specified rate by the
     * main loop.
     */
    virtual void update(const ros::Time& time, const ros::Duration& period);
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
