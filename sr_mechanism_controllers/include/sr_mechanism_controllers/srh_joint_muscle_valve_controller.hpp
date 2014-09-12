/**
 * @file   srh_joint_muscle_valve_controller.hpp
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
 * @brief Compute an effort demand from the effort error. As the
 *  effort PID loop is running on the motor boards, there's no PID
 *  loops involved here. We're just using the friction compensation
 *  algorithm to take into account the friction of the tendons.
 *
 */


#ifndef _SRH_MUSCLE_VALVE_CONTROLLER_HPP_
#define _SRH_MUSCLE_VALVE_CONTROLLER_HPP_

#include <sr_mechanism_controllers/sr_controller.hpp>
#include <sr_robot_msgs/JointMuscleValveControllerState.h>
#include <sr_robot_msgs/JointMuscleValveControllerCommand.h>

namespace controller
{
  class SrhJointMuscleValveController : public SrController
  {
  public:
    SrhJointMuscleValveController();

    bool init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n);

    virtual void starting(const ros::Time& time);

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update(const ros::Time& time, const ros::Duration& period);

    virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    virtual bool resetGains(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
    //bool setGains(sr_robot_msgs::SetEffortControllerGains::Request &req, sr_robot_msgs::SetEffortControllerGains::Response &resp);

    int8_t cmd_valve_muscle_[2];                            /**< Last commanded valve values for each muscle. */
    unsigned int cmd_duration_ms_[2];                      /**< Last duration commanded for the valve command for each muscle. */
    unsigned int current_duration_ms_[2];                  // These are the actual counters used to maintain the commanded value for the valve during the commanded time

    int8_t cmd_valve_muscle_min_;
    int8_t cmd_valve_muscle_max_;

  private:
    //publish our joint controller state
    boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::JointMuscleValveControllerState> > controller_state_publisher_;

    ros::Subscriber sub_command_;
    void setCommandCB(const sr_robot_msgs::JointMuscleValveControllerCommandConstPtr& msg);

    ///read all the controller settings from the parameter server
    void read_parameters();

    /// enforce that the value of the received command is in the allowed range
    int8_t clamp_command( int8_t cmd );
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
