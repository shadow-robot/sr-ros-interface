#include "sr_standalone/sr_ros_wrapper.hpp"
#include <std_msgs/Float64.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <boost/algorithm/string/case_conv.hpp>
using namespace std;
using boost::algorithm::to_lower_copy;

namespace shadow_robot_standalone
{

ShadowHand::SrRosWrapper::SrRosWrapper(int argc, char **argv)
{
  // Must call ros::init() before creating the first NodeHandle.
  ros::init(argc, argv, "sh_standalone_node");
  nh_.reset(new ros::NodeHandle());
  n_tilde_.reset(new ros::NodeHandle("~"));

  string joint_states_topic;
  n_tilde_->searchParam("prefix", joint_states_topic);
  joint_states_topic += "/joint_states";
  ROS_INFO_STREAM("joint_states_topic = " << joint_states_topic);

  string tactile_topic;
  n_tilde_->searchParam("prefix", tactile_topic);
  tactile_topic += "/tactile";
  ROS_INFO_STREAM("tactile_topic = " << tactile_topic);

  joint_states_sub_ = nh_->subscribe(joint_states_topic, 1, &SrRosWrapper::joint_state_cb, this);

  tactile_sub_ = nh_->subscribe(tactile_topic, 1, &SrRosWrapper::tactile_cb, this);

  hand_commander_.reset(new shadowrobot::HandCommander());
  
  const char joints[24][5] = {"WRJ2", "WRJ1", "FFJ4", "FFJ3", "FFJ1", "FFJ2", "MFJ4", "MFJ3", "MFJ1",
                              "MFJ2", "RFJ4", "RFJ3", "RFJ1", "RFJ2", "LFJ5", "LFJ4", "LFJ3", "LFJ1",
                              "LFJ2", "THJ5", "THJ4", "THJ3", "THJ2", "THJ1"};
  for (size_t i = 0; i < 24; ++i)
  {
    string joint = joints[i];
    string pos_ctrl_name = "/sh_" + to_lower_copy(joint) + "_position_controller";
    pr2_mechanism_msgs::LoadController pos_to_load;
    pos_to_load.request.name = pos_ctrl_name;
    if (ros::service::call("pr2_controller_manager/load_controller", pos_to_load))
      ROS_INFO_STREAM("loaded " << pos_ctrl_name);
    else
      ROS_INFO_STREAM("failed " << pos_ctrl_name);

    string eff_ctrl_name = "/sh_" + to_lower_copy(joint) + "_effort_controller";
    pr2_mechanism_msgs::LoadController eff_to_load;
    eff_to_load.request.name = eff_ctrl_name;
    ros::service::call("pr2_controller_manager/load_controller", eff_to_load);

    string topic_name = eff_ctrl_name + "/command";
    torque_pubs_[joint] = nh_->advertise<std_msgs::Float64>(topic_name, 1, true);
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

bool ShadowHand::SrRosWrapper::get_control_type(ControlType & current_ctrl_type)
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

bool ShadowHand::SrRosWrapper::set_control_type(const ControlType & new_ctrl_type)
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

    for (vector<string>::const_iterator it = joint_states_.names.begin(); it != joint_states_.names.end(); ++it)
    {
      string pos_ctrl_name = "/sh_" + to_lower_copy(*it) + "_position_controller";
      string eff_ctrl_name = "/sh_" + to_lower_copy(*it) + "_effort_controller";
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
  vector<sr_robot_msgs::joint> joint_commands;
  sr_robot_msgs::joint joint_command;
  joint_command.joint_name = joint_name;
  joint_command.joint_target = target;
  joint_commands.push_back(joint_command);
  hand_commander_->sendCommands(joint_commands);
  spin();
}

void ShadowHand::SrRosWrapper::send_torque(const string &joint_name, double target)
{
  if (torque_pubs_.count(joint_name))
  {
    std_msgs::Float64 msg;
    msg.data = target;
    torque_pubs_[joint_name].publish(msg);
  }
  spin();
}

void ShadowHand::SrRosWrapper::joint_state_cb(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_states_.names = msg->name;
  joint_states_.positions = msg->position;
  joint_states_.velocities = msg->velocity;
  joint_states_.efforts = msg->effort;
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
