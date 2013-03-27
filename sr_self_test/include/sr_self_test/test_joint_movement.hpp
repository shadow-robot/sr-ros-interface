/**
 * @file   test_joint_movement.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Mar 27 05:47:00 2013
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
 * @brief  Class used for testing the controllers of a given joint.
 *
 *
 */

#ifndef _TEST_JOINT_MOVEMENT_HPP_
#define _TEST_JOINT_MOVEMENT_HPP_

#include <sr_movements/movement_from_image.hpp>
#include <sr_movements/movement_publisher.hpp>
#include <sr_robot_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

namespace shadow_robot
{
  class TestJointMovement
  {
  public:
    TestJointMovement(std::string joint_name);
    ~TestJointMovement() {};

    double mse;
    std::map<std::string, std::vector<double> > values;

  private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    ros::Subscriber sub_state_;
    void state_cb_(const sr_robot_msgs::JointControllerState::ConstPtr& msg);

    ros::Subscriber mse_sub_;
    void mse_cb_(const std_msgs::Float64::ConstPtr& msg);

    boost::shared_ptr<shadowrobot::MovementFromImage> mvt_from_img_;
    boost::shared_ptr<shadowrobot::MovementPublisher> mvt_pub_;

    ros::NodeHandle nh_tilde_;

    boost::shared_ptr<boost::thread> thread_;

    std::string joint_name_;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _TEST_JOINT_MOVEMENT_HPP_ */
