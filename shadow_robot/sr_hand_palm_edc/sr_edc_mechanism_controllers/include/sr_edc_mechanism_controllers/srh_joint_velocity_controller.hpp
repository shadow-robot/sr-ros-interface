/**
 * @file   srh_joint_velocity_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Aug 17 12:32:01 2011
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
 * @brief  Follows a velocity target. The velocity demand is converted into a force
 * demand by a PID loop.
 *
 */


#ifndef _SRH_JOINT_VELOCITY_CONTROLLER_HPP_
#define _SRH_JOINT_VELOCITY_CONTROLLER_HPP_

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

#include <sr_edc_mechanism_controllers/sr_friction_compensation.hpp>


namespace controller
{

  class SrhJointVelocityController : public pr2_controller_interface::Controller
  {
  public:

    SrhJointVelocityController();
    ~SrhJointVelocityController();

    bool init( pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
               const control_toolbox::Pid &pid_velocity);
    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

    /*!
     * \brief Give set position of the joint for next update: revolute (angle) and prismatic (position)
     *
     * \param command
     */
    void setCommand(double cmd);

    /*!
     * \brief Get latest position command to the joint: revolute (angle) and prismatic (position).
     */
    void getCommand(double & cmd);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    bool setGains(sr_robot_msgs::SetPidGains::Request &req, sr_robot_msgs::SetPidGains::Response &resp);

    std::string getJointName();
    pr2_mechanism_model::JointState *joint_state_;        /**< Joint we're controlling. */
    ros::Duration dt_;
    double command_;                            /**< Last commanded position. */

  private:
    int loop_count_;
    bool initialized_;
    pr2_mechanism_model::RobotState *robot_;              /**< Pointer to robot structure. */
    control_toolbox::Pid pid_controller_velocity_;       /**< Internal PID controller for the velocity loop. */
    ros::Time last_time_;                          /**< Last time stamp of update. */

    ros::NodeHandle node_, n_tilde_;

    boost::scoped_ptr<
      realtime_tools::RealtimePublisher<
        pr2_controllers_msgs::JointControllerState> > controller_state_publisher_ ;

    boost::shared_ptr<sr_friction_compensation::SrFrictionCompensator> friction_compensator;

    ros::Subscriber sub_command_;
    void setCommandCB(const std_msgs::Float64ConstPtr& msg);

    ros::ServiceServer serve_set_gains_;

    ///clamps the force demand to this value
    double max_force_demand;
    ///the deadband for the friction compensation algorithm
    int friction_deadband;
    ///the velocity deadband value used in the hysteresis_deadband
    double velocity_deadband;

    ///We're using an hysteresis deadband.
    sr_deadband::HysteresisDeadband<double> hysteresis_deadband;

    ///read all the controller settings from the parameter server
    void read_parameters();
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
