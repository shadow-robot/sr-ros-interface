/**
 * @file   sr_controller.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Aug 25 12:29:38 2011
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
 * @brief   A generic controller for the Shadow Robot EtherCAT hand's joints.
 *
 */

#include "sr_mechanism_controllers/sr_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

using namespace std;

namespace controller {

  SrController::SrController()
    : joint_state_(NULL),
      joint_state_2(NULL),
      has_j2(false),
      command_(0),
      min_(0.0),
      max_(sr_math_utils::pi),
      loop_count_(0),
      initialized_(false),
      robot_(NULL),
      n_tilde_("~"),
      max_force_demand(1023.),
      friction_deadband(5),
      max_force_factor_(1.0)
  {
  }

  SrController::~SrController()
  {
    sub_command_.shutdown();
  }

  bool SrController::is_joint_0()
  {
    // joint_name_ has unknown length
    // it is assumed that last char is the joint number
    if (joint_name_[joint_name_.size()-1] == '0')
      return true;
    return false;
  }

  void SrController::get_joints_states_1_2()
  {
    string j1 = joint_name_, j2 = joint_name_;
    j1[j1.size()-1] = '1';
    j2[j2.size()-1] = '2';

    ROS_DEBUG_STREAM("Joint 0: " << j1 << " " << j2);

    joint_state_ = robot_->getJointState(j1);
    joint_state_2 = robot_->getJointState(j2);
  }

  void SrController::after_init()
  {
    sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &SrController::setCommandCB, this);
    sub_max_force_factor_ = node_.subscribe<std_msgs::Float64>("max_force_factor", 1, &SrController::maxForceFactorCB, this);
  }

  std::string SrController::getJointName()
  {
    return joint_state_->joint_->name;
  }

  void SrController::get_min_max( urdf::Model model, std::string joint_name )
  {
    if (joint_name_[joint_name.size() - 1]  ==  '0')
    {
      joint_name[joint_name.size() - 1] = '1';
      std::string j1 = joint_name;
      joint_name[joint_name.size() - 1] = '2';
      std::string j2 = joint_name;

      boost::shared_ptr<const urdf::Joint> joint1 = model.getJoint( j1 );
      boost::shared_ptr<const urdf::Joint> joint2 = model.getJoint( j2 );

      min_ = joint1->limits->lower + joint2->limits->lower;
      max_ = joint1->limits->upper + joint2->limits->upper;
    }
    else
    {
      boost::shared_ptr<const urdf::Joint> joint = model.getJoint( joint_name );

      min_ = joint->limits->lower;
      max_ = joint->limits->upper;
    }
  }

  double SrController::clamp_command( double cmd )
  {
    if(cmd < min_)
      return min_;

    if(cmd > max_)
      return max_;

    return cmd;
  }

  void SrController::maxForceFactorCB(const std_msgs::Float64ConstPtr& msg)
  {
    if((msg->data >= 0.0) && (msg->data <= 1.0))
    {
      max_force_factor_ = msg->data;
    }
    else
    {
      ROS_ERROR("Max force factor must be between 0.0 and 1.0. Discarding received value: %f", msg->data);
    }
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
