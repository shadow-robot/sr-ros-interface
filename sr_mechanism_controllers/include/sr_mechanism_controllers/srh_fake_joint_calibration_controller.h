/**
 * @file   srh_fake_joint_calibration_controller.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Aug 23 12:03:37 2011
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
 * @brief  A Fake joint calibration controller. Only loads the force pid settings
 * from the parameter server.
 *
 *
 */

#ifndef _SRH_FAKE_JOINT_CALIBRATION_CONTROLLER_
#define _SRH_FAKE_JOINT_CALIBRATION_CONTROLLER_

#include "ros/node_handle.h"
#include "ros_ethercat_model/robot_state.hpp"
#include "velocity_controllers/joint_velocity_controller.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/Empty.h"
#include "controller_interface/controller.h"
#include <boost/smart_ptr.hpp>


namespace controller
{

  class SrhFakeJointCalibrationController : public controller_interface::Controller<ros_ethercat_model::RobotState>
  {
  public:
    SrhFakeJointCalibrationController();

    virtual bool init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n);

    virtual void update(const ros::Time& time, const ros::Duration& period);

    bool calibrated() { return calibration_state_ == CALIBRATED; }
    void beginCalibration()
    {
      if (calibration_state_ == IS_INITIALIZED)
        calibration_state_ = BEGINNING;
    }

  protected:

    ros_ethercat_model::RobotState* robot_;
    ros::NodeHandle node_;
    boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Empty> > pub_calibrated_;
    ros::Time last_publish_time_;

    enum { IS_INITIALIZED, BEGINNING, MOVING_TO_LOW, MOVING_TO_HIGH, CALIBRATED };
    int calibration_state_;

    ros_ethercat_model::Actuator *actuator_;
    ros_ethercat_model::JointState *joint_;

    std::string joint_name_, actuator_name_, robot_id_, joint_prefix_, ns_;

    /**
     * Read the pids values from the parameter server and calls the service
     * to set them on the hand.
     */
    void initialize_pids();
  };

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif
