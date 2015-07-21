
#ifndef SR_ROS_INTERFACE_GAZEBO_HARDWARE_SIM_H
#define SR_ROS_INTERFACE_GAZEBO_HARDWARE_SIM_H

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include "ros_ethercat_model/robot_state.hpp"

// gazebo_ros_control
#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
#include <urdf/model.h>

namespace sr_gazebo_sim {

class SrGazeboHWSim : public gazebo_ros_control::DefaultRobotHWSim {
public:

  SrGazeboHWSim();

  bool initSim(
      const std::string &robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model *const urdf_model,
      std::vector <transmission_interface::TransmissionInfo> transmissions);

  void readSim(ros::Time time, ros::Duration period);

  void writeSim(ros::Time time, ros::Duration period);

protected:

  template <class T>
  void fixJointName(std::vector<T> *items, const std::string old_joint_name, const std::string new_joint_name) const;

  bool isFourFingersJoints(const std::string joint_name, const unsigned joint_index) const;

  void addFakeTransmissionsForJ0(std::vector<transmission_interface::TransmissionInfo> *transmissions) const;

  void initializeFakeRobotState(const urdf::Model*const urdf_model);

  ros_ethercat_model::RobotState fake_state_;

};


typedef boost::shared_ptr <SrGazeboHWSim> SrGazeboHWSimPtr;

}


#endif //SR_ROS_INTERFACE_GAZEBO_HARDWARE_SIM_H
