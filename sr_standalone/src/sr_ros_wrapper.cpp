#include "sr_standalone/sr_ros_wrapper.hpp"
#include <std_msgs/Float64.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <boost/algorithm/string/case_conv.hpp>
#include <algorithm>

using namespace std;
using boost::algorithm::to_upper_copy;
static const size_t JOINTS_WITH_STATE = 20;

namespace shadow_robot_standalone
{

static const string ctrl_joints[JOINTS_WITH_STATE] = {"ffj0", "ffj3", "ffj4",
                                                      "lfj0", "lfj3", "lfj4", "lfj5",
                                                      "mfj0", "mfj3", "mfj4",
                                                      "rfj0", "rfj3", "rfj4",
                                                      "thj1", "thj2", "thj3", "thj4", "thj5",
                                                      "wrj1", "wrj2"};

ShadowHand::SrRosWrapper::SrRosWrapper()
{
  int _argc = 0;
  char **_argv = NULL;
  ros::init(_argc, _argv, "sh_standalone_node");
  nh_.reset(new ros::NodeHandle());
  n_tilde_.reset(new ros::NodeHandle("~"));

  string pr;
  n_tilde_->searchParam("prefix", pr);

  joint_states_sub_ = nh_->subscribe(pr + "/joint_states", 1, &SrRosWrapper::joint_state_cb, this);
  joint0_states_sub_ = nh_->subscribe(pr + "joint_0s/joint_states", 1, &SrRosWrapper::joint_state_cb, this);
  tactile_sub_ = nh_->subscribe(pr + "/tactile", 1, &SrRosWrapper::tactile_cb, this);

  hand_commander_.reset(new shadowrobot::HandCommander());

  for (size_t i = 0; i < JOINTS_WITH_STATE; ++i)
  {
    pr2_mechanism_msgs::LoadController pos_to_load;
    pos_to_load.request.name = string("/sh_") + ctrl_joints[i] + "_position_controller";
    ros::service::call("pr2_controller_manager/load_controller", pos_to_load);

    pr2_mechanism_msgs::LoadController eff_to_load;
    eff_to_load.request.name = string("/sh_") + ctrl_joints[i] + "_effort_controller";
    ros::service::call("pr2_controller_manager/load_controller", eff_to_load);

    torque_pubs_[to_upper_copy(ctrl_joints[i])] =
      nh_->advertise<std_msgs::Float64>(eff_to_load.request.name + "/command", 1, true);
  }
}

void ShadowHand::SrRosWrapper::spin(void)
{
  if (ros::ok())
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
}

bool ShadowHand::SrRosWrapper::get_control_type(ControlType &current_ctrl_type)
{
  spin();
  sr_robot_msgs::ChangeControlType change_ctrl_type;
  change_ctrl_type.request.control_type.control_type = sr_robot_msgs::ControlType::QUERY;
  if (ros::service::call("realtime_loop/change_control_type", change_ctrl_type))
  {
    if (change_ctrl_type.response.result.control_type == sr_robot_msgs::ControlType::PWM)
    {
      current_ctrl_type = POSITION_PWM;
      return true;
    }
    else if (change_ctrl_type.response.result.control_type == sr_robot_msgs::ControlType::FORCE)
    {
      current_ctrl_type = EFFORT_TORQUE;
      return true;
    }
  }

  ROS_ERROR_STREAM("Failed to get current control type.");
  return false;
}

bool ShadowHand::SrRosWrapper::set_control_type(const ControlType &new_ctrl_type)
{
  sr_robot_msgs::ChangeControlType change_ctrl_type;
  if (new_ctrl_type == POSITION_PWM)
    change_ctrl_type.request.control_type.control_type = sr_robot_msgs::ControlType::PWM;
  else if (new_ctrl_type == EFFORT_TORQUE)
    change_ctrl_type.request.control_type.control_type = sr_robot_msgs::ControlType::FORCE;
  else
  {
    ROS_ERROR_STREAM("Unknown control type: " << new_ctrl_type);
    return false;
  }
  if (!ros::service::call("realtime_loop/change_control_type", change_ctrl_type))
  {
    ROS_ERROR_STREAM("Failed to change control type to " << new_ctrl_type);
    return false;
  }

  sleep(3);

  ControlType current_ctrl_type;
  if (get_control_type(current_ctrl_type) && current_ctrl_type == new_ctrl_type)
  {
    pr2_mechanism_msgs::SwitchController cswitch;
    cswitch.request.strictness = pr2_mechanism_msgs::SwitchController::Request::STRICT;

    for (size_t i = 0; i < JOINTS_WITH_STATE; ++i)
    {
      string pos_ctrl_name = "/sh_" + ctrl_joints[i] + "_position_controller";
      string eff_ctrl_name = "/sh_" + ctrl_joints[i] + "_effort_controller";
      if (current_ctrl_type == POSITION_PWM)
      {
        cswitch.request.start_controllers.push_back(pos_ctrl_name);
        cswitch.request.stop_controllers.push_back(eff_ctrl_name);
      }
      else if (current_ctrl_type == EFFORT_TORQUE)
      {
        cswitch.request.start_controllers.push_back(eff_ctrl_name);
        cswitch.request.stop_controllers.push_back(pos_ctrl_name);
      }
      if (ros::service::call("pr2_controller_manager/switch_controller", cswitch))
        ROS_INFO("switched controllers");
      else
        ROS_INFO("failed on switching");
    }
    return true;
  }

  ROS_ERROR_STREAM("Failed to change control type to " << new_ctrl_type);
  return false;
}

void ShadowHand::SrRosWrapper::send_position(const string &joint_name, double target)
{
  if (torque_pubs_.count(joint_name) == 0)
  {
    ROS_ERROR_STREAM("Unknown joint name : " << joint_name);
    return;
  }

  sr_robot_msgs::joint joint_command;
  joint_command.joint_name = joint_name;
  joint_command.joint_target = target * (180 / M_PI); // convert to degrees
  hand_commander_->sendCommands(vector<sr_robot_msgs::joint>(1, joint_command));
  spin();
}

void ShadowHand::SrRosWrapper::send_all_positions(const vector<double> &targets)
{
  if (targets.size() != torque_pubs_.size())
  {
    ROS_ERROR_STREAM("targets size should be " << torque_pubs_.size());
    return;
  }

  vector<sr_robot_msgs::joint> joint_commands;
  sr_robot_msgs::joint joint_command;

  boost::unordered_map<string, ros::Publisher>::const_iterator pit = torque_pubs_.begin();
  vector<double>::const_iterator tit = targets.begin();
  while (pit != torque_pubs_.end())
  {
    joint_command.joint_name = pit->first;
    joint_command.joint_target = *tit * (180 / M_PI); // convert to degrees
    joint_commands.push_back(joint_command);

    ++pit;
    ++tit;
  }
  hand_commander_->sendCommands(joint_commands);
  spin();
}

void ShadowHand::SrRosWrapper::send_torque(const string &joint_name, double target)
{
  if (!torque_pubs_.count(joint_name))
  {
    ROS_ERROR_STREAM("Unknown joint name : " << joint_name);
    return;
  }

  std_msgs::Float64 msg;
  msg.data = target;
  torque_pubs_[joint_name].publish(msg);
  spin();
}

void ShadowHand::SrRosWrapper::send_all_torques(const vector<double> &targets)
{
  if (targets.size() != torque_pubs_.size())
  {
    ROS_ERROR_STREAM("targets size should be " << torque_pubs_.size());
    return;
  }

  boost::unordered_map<string, ros::Publisher>::const_iterator pit = torque_pubs_.begin();
  vector<double>::const_iterator tit = targets.begin();
  while (pit != torque_pubs_.end())
  {
    std_msgs::Float64 msg;
    msg.data = *tit;
    pit->second.publish(msg);

    ++pit;
    ++tit;
  }
  spin();
}

void ShadowHand::SrRosWrapper::joint_state_cb(const sensor_msgs::JointStateConstPtr& msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i)
    joint_states_[msg->name[i]] = JointState(msg->position[i], msg->velocity[i], msg->effort[i]);
}

void ShadowHand::SrRosWrapper::tactile_cb(const sr_robot_msgs::BiotacAllConstPtr& msg)
{
  //initialise the vector to the correct size if empty (first time)
  if (tactiles_.empty())
    tactiles_.resize(msg->tactiles.size());

  //fills the data with the incoming biotacs
  for (size_t i = 0; i < tactiles_.size(); ++i)
  {
    tactiles_[i].pac0 = msg->tactiles[i].pac0;
    tactiles_[i].pac1 = msg->tactiles[i].pac1;
    tactiles_[i].pdc = msg->tactiles[i].pdc;

    tactiles_[i].tac = msg->tactiles[i].tac;
    tactiles_[i].tdc = msg->tactiles[i].tdc;

    if (Tactile::no_of_electrodes == msg->tactiles[i].electrodes.size())
    {
      for (size_t elec_i = 0; elec_i < msg->tactiles[i].electrodes.size(); ++elec_i)
        tactiles_[i].electrodes[elec_i] = msg->tactiles[i].electrodes[elec_i];
    }
    else
    {
      ROS_ERROR_STREAM("Verify the size of msg->tactiles[i].electrodes. \n"
                       << "It is " << msg->tactiles[i].electrodes.size()
                       << ", but it should be " << Tactile::no_of_electrodes << ".");
    }
  }
}

} // namespace
