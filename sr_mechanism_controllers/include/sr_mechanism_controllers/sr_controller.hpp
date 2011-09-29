/**
 * @file   sr_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Aug 25 12:22:47 2011
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
 * @brief  A generic controller for the Shadow Robot EtherCAT hand's joints.
 *
 */


#ifndef _SRH_CONTROLLER_HPP_
#define _SRH_CONTROLLER_HPP_

#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <pr2_controllers_msgs/JointControllerState.h>

#include <utility>

#include <sr_robot_msgs/SetPidGains.h>

#include <sr_utilities/sr_deadband.hpp>

#include <sr_mechanism_controllers/sr_friction_compensation.hpp>


namespace controller
{

  class SrController : public pr2_controller_interface::Controller
  {
  public:

    SrController();
    virtual ~SrController();

    virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

    /*!
     * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
     *
     * \param command
     */
    void setCommand(double cmd);
    void setCommandCB(const std_msgs::Float64ConstPtr& msg);

    /*!
     * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
     */
    void getCommand(double & cmd);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

    std::string getJointName();
    pr2_mechanism_model::JointState *joint_state_;        /**< Joint we're controlling. */
    pr2_mechanism_model::JointState *joint_state_2;        /**< 2ndJoint we're controlling if joint 0. */
    bool has_j2;         /**< true if this is a joint 0. */
    ros::Duration dt_;
    double command_;                            /**< Last commanded position. */

  protected:
    ///call this function at the end of the init function in the inheriting classes.
    void after_init();

    /**
     * Clamps the command to the correct range
     * (between min and max).
     *
     * @param cmd the command we want to clamp
     *
     * @return the clamped command
     */
    double clamp_command( double cmd );

    /**
     * Reads the min and max from the robot model for
     * the current joint.
     *
     * @param model the urdf description of the robot
     * @param joint_name the name of the joint
     */
    void get_min_max( urdf::Model model, std::string joint_name );

    ///Min and max range of the joint, used to clamp the command.
    double min_, max_;

    int loop_count_;
    bool initialized_;
    pr2_mechanism_model::RobotState *robot_;              /**< Pointer to robot structure. */
    ros::Time last_time_;                          /**< Last time stamp of update. */

    ros::NodeHandle node_, n_tilde_;

    boost::scoped_ptr<
      realtime_tools::RealtimePublisher<
        pr2_controllers_msgs::JointControllerState> > controller_state_publisher_ ;

    boost::shared_ptr<sr_friction_compensation::SrFrictionCompensator> friction_compensator;

    ros::Subscriber sub_command_;
    ros::ServiceServer serve_set_gains_;

    ///clamps the force demand to this value
    double max_force_demand;
    ///the deadband for the friction compensation algorithm
    int friction_deadband;

    ///We're using an hysteresis deadband.
    sr_deadband::HysteresisDeadband<double> hysteresis_deadband;
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
