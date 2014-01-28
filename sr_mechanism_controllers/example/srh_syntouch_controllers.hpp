/**
 * @file   srh_syntouch_controllers.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Dec  6 11:56:49 2011
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
 * @brief Dummy controller to show how to use the biotac tactiles to compute the force demand.
 *
 */


#ifndef SRH_SYNTOUCH_CONTROLLER_H
#define SRH_SYNTOUCH_CONTROLLER_H

#include <sr_mechanism_controllers/sr_controller.hpp>
#include <sr_robot_msgs/JointControllerState.h>
#include <sr_hardware_interface/tactile_sensors.hpp>
#include <sr_hardware_interface/sr_actuator.hpp>

namespace controller
{
  class SrhSyntouchController : public SrController
  {
  public:

    SrhSyntouchController();
    ~SrhSyntouchController();

    bool init( pr2_mechanism_model::RobotState *robot, const std::string &joint_name );
    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

  private:
    //publish our joint controller state
    boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::JointControllerState> > controller_state_publisher_;

    sr_actuator::SrActuator* actuator_;
  };
} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
