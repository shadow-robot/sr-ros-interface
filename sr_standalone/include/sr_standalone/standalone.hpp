#pragma once

#include <string>
#include <vector>
#include <boost/scoped_ptr.hpp>

namespace shadow_robot_standalone
{

struct JointStates
{
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
};

struct Tactile
{
  int pac0;
  int pac1;
  int pdc;

  int tac;
  int tdc;

  int electrodes[19];
};

enum ControlType
  {
    POSITION_PWM,
    EFFORT_TORQUE
  };

class ShadowHand
{
public:
  ShadowHand(int argc, char** argv);
  ~ShadowHand();

  /**
   * Set the control type to be used on the hand.
   *
   * @param control_type Either position control over PWM, or
   *        effort control over Torque.
   *
   * @return true if success.
   */
  bool set_control_type(ControlType control_type);

  /**
   * Send a position target, in radians, to the given joints
   * position controller.
   *
   * @param joint_name Name of the joint to control
   * @param a position in radians
   */
  void send_position(const std::string &joint_name, double target);

  /**
   * Send a torque target, to the given joint.
   *
   * @param joint_name Name of the joint to control
   * @param a torque target.
   */
  void send_torque(const std::string &joint_name, double target);

  /**
   * Retrieves the latest information about the joints.
   * vectors will be empty if nothing has been published
   *
   * @return Struct containing all the joints positions,
   *         velocity and effort.
   */
  JointStates get_joint_states() const;
  /**
   * Retrieves the tactile data from the biotacs.
   *
   * @return A vector of tactiles in the following order:
   *         FF, MF, RF, LF, TH
   */
  std::vector<Tactile> get_tactiles() const;

  /**
   * Get a list of all the joint names in the hand.
   *
   * @return vector containing all the hand joint names
   */
  std::vector<std::string> get_list_of_joints() const;

private:
  /*
   * Pimpl idiom (also called Handle/Body idiom):
   * for hiding implementation details in the header file.
   */
  class SrRosWrapper; // fwd declaration
  boost::scoped_ptr<SrRosWrapper> wrapper_;
};

} // namespace
