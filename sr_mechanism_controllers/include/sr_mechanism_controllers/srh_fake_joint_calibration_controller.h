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
#include "pr2_mechanism_model/robot.h"
#include "robot_mechanism_controllers/joint_velocity_controller.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/Empty.h"


namespace controller
{

  class SrhFakeJointCalibrationController : public pr2_controller_interface::Controller
  {
  public:
    SrhFakeJointCalibrationController();
    virtual ~SrhFakeJointCalibrationController();

    virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

    virtual void update();

    bool calibrated() { return state_ == CALIBRATED; }
    void beginCalibration()
    {
      if (state_ == IS_INITIALIZED)
        state_ = BEGINNING;
    }

  protected:

    pr2_mechanism_model::RobotState* robot_;
    ros::NodeHandle node_;
    boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Empty> > pub_calibrated_;
    ros::Time last_publish_time_;

    enum { IS_INITIALIZED, BEGINNING, MOVING_TO_LOW, MOVING_TO_HIGH, CALIBRATED };
    int state_;
    int countdown_;

    double search_velocity_, reference_position_;
    bool original_switch_state_;

    pr2_hardware_interface::Actuator *actuator_;
    pr2_mechanism_model::JointState *joint_;
    boost::shared_ptr<pr2_mechanism_model::Transmission> transmission_;

    std::string joint_name_, actuator_name_;

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
