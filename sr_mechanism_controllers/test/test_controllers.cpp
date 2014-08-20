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
#include "sr_mechanism_controllers/test/test_controllers.hpp"

TestControllers::TestControllers()
{
}

void TestControllers::init()
{
  hw = new hardware_interface::HardwareInterface();

  model = new TiXmlDocument();

  ros::NodeHandle rosnode;
  std::string urdf_param_name;
  std::string urdf_string;
  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    ROS_DEBUG("Waiting for urdf: %s on the param server.", "sh_description");
    if (rosnode.searchParam("sh_description", urdf_param_name))
    {
      rosnode.getParam(urdf_param_name, urdf_string);
      ROS_DEBUG("found upstream\n%s\n------\n%s\n------\n%s", "sh_description", urdf_param_name.c_str(), urdf_string.c_str());
    }
    else
    {
      rosnode.getParam("sh_description", urdf_string);
      ROS_DEBUG("found in node namespace\n%s\n------\n%s\n------\n%s", "sh_description", urdf_param_name.c_str(), urdf_string.c_str());
    }
    usleep(100000);
  }
  ROS_DEBUG("Parsing xml...");

  // initialize TiXmlDocument doc with a string
  if (!model->Parse(urdf_string.c_str()) && model->Error())
  {
    ROS_ERROR("Failed to parse urdf: %s\n",
              urdf_string.c_str());
  }
  else
  {
    robot_state = new ros_ethercat_model::RobotState(model->RootElement());

    //add a fake FFJ3 transmission
    transmission.name_ = "FFJ3";
    transmission.actuator_ = new sr_actuator::SrMotorActuator();
    transmission.actuator_->name_ = "FFJ3";
    robot_state->transmissions_.push_back(&transmission);

    joint_state = robot_state->getJointState("FFJ3");
    joint_state->calibrated_ = true;

    init_controller();
    controller->starting(ros::Time::now());

    //initialize the controller:
    joint_state->position_ = 0.0;
    controller->update(ros::Time::now(), ros::Duration(0.01));
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
 */
