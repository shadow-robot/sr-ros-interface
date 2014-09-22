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

#include <controller_interface/controller.h>
#include <ros_ethercat_model/robot_state.hpp>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <control_msgs/JointControllerState.h>

#include <utility>

#include <sr_robot_msgs/SetPidGains.h>

#include <sr_utilities/sr_deadband.hpp>

#include <sr_mechanism_controllers/sr_friction_compensation.hpp>


namespace controller
{

  class SrController : public controller_interface::Controller<ros_ethercat_model::RobotState>
  {
  public:

    SrController();
    virtual ~SrController();

    virtual bool init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n) = 0;

    virtual void starting(const ros::Time& time) {};

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update(const ros::Time& time, const ros::Duration& period) = 0;

    virtual bool resetGains(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) { return true; };

    virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min) {};

    std::string getJointName();
    ros_ethercat_model::JointState *joint_state_;   /**< Joint we're controlling. */
    ros_ethercat_model::JointState *joint_state_2;  /**< 2ndJoint we're controlling if joint 0. */
    bool has_j2;                                    /**< true if this is a joint 0. */
    double command_;                                /**< Last commanded position. */

  protected:
    // true if this is joint 0
    bool is_joint_0();

    // set joint_state_ and joint_state_2
    void get_joints_states_1_2();

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
    virtual double clamp_command( double cmd );

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
    ros_ethercat_model::RobotState *robot_;              /**< Pointer to robot structure. */

    ros::NodeHandle node_, n_tilde_;
    std::string joint_name_;

    boost::scoped_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_ ;

    boost::scoped_ptr<sr_friction_compensation::SrFrictionCompensator> friction_compensator;

    ///set the command from a topic
    virtual void setCommandCB(const std_msgs::Float64ConstPtr& msg) {}
    ros::Subscriber sub_command_;
    ros::ServiceServer serve_set_gains_;
    ros::ServiceServer serve_reset_gains_;

    ///clamps the force demand to this value
    double max_force_demand;
    ///the deadband for the friction compensation algorithm
    int friction_deadband;

    ///We're using an hysteresis deadband.
    sr_deadband::HysteresisDeadband<double> hysteresis_deadband;

    //The max force factor is a number between 0.0 and 1.0 that will multiply the max_force_demand
    //it is initialized to 1.0 and can be updated via a topic.
    //This is intended to be used e.g. as part of a hand self protection mechanism, where max_force is reduced in certain cases
    double max_force_factor_;
    ros::Subscriber sub_max_force_factor_;
    /**
     * Callback function for the max force factor topic
     *
     * @param msg the message receiver over the topic
     */
    void maxForceFactorCB(const std_msgs::Float64ConstPtr& msg);
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
