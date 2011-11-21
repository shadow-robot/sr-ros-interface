/**
 * @file   srh_effort_joint_controller.hpp
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


#ifndef _SRH_EFFORT_CONTROLLER_HPP_
#define _SRH_EFFORT_CONTROLLER_HPP_

#include <sr_mechanism_controllers/sr_controller.hpp>
#include <sr_robot_msgs/SetEffortControllerGains.h>

namespace controller
{
  class SrhEffortJointController : public SrController
  {
  public:
    SrhEffortJointController();
    ~SrhEffortJointController();

    bool init( pr2_mechanism_model::RobotState *robot, const std::string &joint_name);
    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    virtual void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    bool setGains(sr_robot_msgs::SetEffortControllerGains::Request &req, sr_robot_msgs::SetEffortControllerGains::Response &resp);

  private:
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
