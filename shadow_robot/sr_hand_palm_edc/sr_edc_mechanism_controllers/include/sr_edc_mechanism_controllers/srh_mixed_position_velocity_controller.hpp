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
 * @brief Compute a velocity demand from the position error:
 *  we use this function (velocity_demand = f(position_error))
 *  to converge smoothly on the position we want.
 *       ____
 *      /
 *     /
 * ___/
 *
 * The velocity demand is then converted into a force demand by a
 * PID loop.
 *
 */


#ifndef SRH_MIXED_POSITION_VELOCITY_CONTROLLER_H
#define SRH_MIXED_POSITION_VELOCITY_CONTROLLER_H

#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <pr2_controllers_msgs/JointControllerState.h>

#include <utility>

#include <sr_robot_msgs/SetMixedPositionVelocityPidGains.h>

#include <sr_utilities/sr_deadband.hpp>

#include <sr_edc_mechanism_controllers/sr_friction_compensation.hpp>


namespace controller
{

  class SrhMixedPositionVelocityJointController : public pr2_controller_interface::Controller
  {
  public:

    SrhMixedPositionVelocityJointController();
    ~SrhMixedPositionVelocityJointController();

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
    bool setGains(sr_robot_msgs::SetMixedPositionVelocityPidGains::Request &req, sr_robot_msgs::SetMixedPositionVelocityPidGains::Response &resp);

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

    /**
     * Compute the velocity demand from the position error:
     *  we use this function (velocity_demand = f(position_error))
     *  to converge smoothly on the position we want.
     *       ____
     *      /
     *     /
     * ___/
     *
     * @param position_error The current position error
     *
     * @return A velocity demand.
     */
    double compute_velocity_demand(double position_error);
    /// The values defining the slope (min and max velocity + slope value)
    double max_velocity_, min_velocity_, slope_velocity_;
    /// those are the X values for the beginning and the end of the slope.
    double max_position_error_, min_position_error_;
    /**
     * Compute the max_position_error and min_position_error_
     * from the max_velocity / min_velocity_, slope_velocity_
     * parameters.
     *
     */
    void set_min_max_position_errors_();
    /// Advertise a service to set the min/max velocity and the slope.
    std::vector<ros::ServiceServer> velocity_services_;

#ifdef DEBUG_PUBLISHER
    ros::Publisher debug_pub;
#endif

    ros::Subscriber sub_command_;
    void setCommandCB(const std_msgs::Float64ConstPtr& msg);

    ros::ServiceServer serve_set_gains_;

    ///clamps the force demand to this value
    double max_force_demand;
    ///the deadband on the position demand
    double position_deadband;
    ///the deadband for the friction compensation algorithm
    int friction_deadband;

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
