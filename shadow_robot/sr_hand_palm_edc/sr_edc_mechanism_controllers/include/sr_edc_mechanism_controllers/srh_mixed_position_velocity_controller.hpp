/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef SRH_MIXED_POSITION_VELOCITY_CONTROLLER_H
#define SRH_MIXED_POSITION_VELOCITY_CONTROLLER_H

/**
   @class pr2_controller_interface::SrhMixedPositionVelocityController
   @brief Joint Position Controller

   This class controls positon using a pid loop.

   @section ROS ROS interface

   @param type Must be "SrhMixedPositionVelocityController"
   @param joint Name of the joint to control.
   @param pid Contains the gains for the PID loop around position.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (std_msgs::Float64) : The joint position to achieve.

   Publishes:

   - @b state (robot_mechanism_controllers::JointControllerState) :
     Current state of the controller, including pid error and gains.

*/

#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <pr2_controllers_msgs/JointControllerState.h>

#include <sr_robot_msgs/SetMixedPositionVelocityPidGains.h>

#include <sr_utilities/calibration.hpp>

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

    double friction_compensation( double position );
    std::vector<joint_calibration::Point> read_friction_map();
    boost::shared_ptr<shadow_robot::JointCalibration> friction_interpoler;

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

    ros::Publisher debug_pub;


    ros::Subscriber sub_command_;
    void setCommandCB(const std_msgs::Float64ConstPtr& msg);

    ros::ServiceServer serve_set_gains_;

    ///clamps the force demand to this value
    double max_force_demand;
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
