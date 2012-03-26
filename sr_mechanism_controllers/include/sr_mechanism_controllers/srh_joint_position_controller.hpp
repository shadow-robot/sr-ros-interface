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
 * @brief  Follows a position target. The position demand is converted into a force
 * demand by a PID loop.
 *
 */


#ifndef _SRH_JOINT_POSITION_CONTROLLER_HPP_
#define _SRH_JOINT_POSITION_CONTROLLER_HPP_

#include <sr_mechanism_controllers/sr_controller.hpp>


namespace controller
{
  class SrhJointPositionController : public SrController
  {
  public:
    SrhJointPositionController();
    ~SrhJointPositionController();

    bool init( pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
               boost::shared_ptr<control_toolbox::Pid> pid_position);
    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    bool setGains(sr_robot_msgs::SetPidGains::Request &req, sr_robot_msgs::SetPidGains::Response &resp);

  private:
    boost::shared_ptr<control_toolbox::Pid> pid_controller_position_;       /**< Internal PID controller for the position loop. */

    ///clamps the force demand to this value
    double max_force_demand;

    ///the position deadband value used in the hysteresis_deadband
    double position_deadband;

    ///We're using an hysteresis deadband.
    sr_deadband::HysteresisDeadband<double> hysteresis_deadband;

    ///read all the controller settings from the parameter server
    void read_parameters();

    std::string joint_name_;
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
