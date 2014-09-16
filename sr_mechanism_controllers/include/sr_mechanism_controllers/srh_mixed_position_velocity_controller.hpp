/**
 * @file   srh_mixed_position_velocity_controller.hpp
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
 * @brief Compute a velocity demand from the position error using
 *        a PID loop.
 *        The velocity demand is then converted into a force demand by a
 *        second PID loop and is sent to the motor.
 *
 */


#ifndef SRH_MIXED_POSITION_VELOCITY_CONTROLLER_H
#define SRH_MIXED_POSITION_VELOCITY_CONTROLLER_H

#include <sr_mechanism_controllers/sr_controller.hpp>
#include <sr_robot_msgs/SetMixedPositionVelocityPidGains.h>
#include <sr_robot_msgs/JointControllerState.h>

namespace controller
{
  class SrhMixedPositionVelocityJointController : public SrController
  {
  public:

    SrhMixedPositionVelocityJointController();

    bool init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n);

    virtual void starting(const ros::Time& time);

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update(const ros::Time& time, const ros::Duration& period);

    virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    virtual void getGains_velocity(double &p, double &i, double &d, double &i_max, double &i_min);
    virtual bool resetGains(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
    bool setGains(sr_robot_msgs::SetMixedPositionVelocityPidGains::Request &req, sr_robot_msgs::SetMixedPositionVelocityPidGains::Response &resp);

  private:
    boost::scoped_ptr<control_toolbox::Pid> pid_controller_position_;       /**< Internal PID controller for the position loop. */
    boost::scoped_ptr<control_toolbox::Pid> pid_controller_velocity_;       /**< Internal PID controller for the velocity loop. */

    //publish our joint controller state
    boost::scoped_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::JointControllerState> > controller_state_publisher_;

    /// The values for the velocity demand saturation
    double max_velocity_, min_velocity_;

#ifdef DEBUG_PUBLISHER
    ros::Publisher debug_pub;
#endif

    //ros::Subscriber sub_command_;
    //void setCommandCB(const std_msgs::Float64ConstPtr& msg);

    ros::ServiceServer serve_set_gains_;

    ///the deadband on the position demand
    double position_deadband;

    ///We're using an hysteresis deadband.
    sr_deadband::HysteresisDeadband<double> hysteresis_deadband;

    ///read all the controller settings from the parameter server
    void read_parameters();

    ///smallest demand we can send to the motors
    int motor_min_force_threshold;

    ///set the position target from a topic
    void setCommandCB(const std_msgs::Float64ConstPtr& msg);

    void resetJointState();
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
