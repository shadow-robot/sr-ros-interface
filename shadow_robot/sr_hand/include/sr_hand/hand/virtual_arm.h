/**
 * @file   virtual_arm.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue Jun 29 14:56:10 2010
 *
 * @brief The virtual arm can be used as a simulator. It modelizes the Shadow Robot muscle arm.
 *
 */

#ifndef VIRTUAL_ARM_H_
#define VIRTUAL_ARM_H_

#include "sr_hand/hand/shadowhand.h"

namespace shadowhand {

class VirtualArm: public Shadowhand
{
public:
  /**
   * Initializes the necessary mappings with a static list of names.
   */
  VirtualArm();
  ///destructor
  ~VirtualArm();

  //virtual classes defined in Shadowhand
  /**
   * This function will set the target of the object to the given angle. It will also set the position to this
   * target.
   *
   * @todo This could be improved by implementing a control algorithm in this theoretic hand.
   *
   * @param joint_name The Joint in joints_map you wish to send the target to.
   * @param target The target in degree
   * @return 0 if success ; -1 if error
   */
  virtual short sendupdate(std::string joint_name, double target);

  /**
   * In the virtual arm, getJointData() simply fetches the data from a given joint in the joints_map. As the targets
   * and positions are up to date, there's no need of reading other values here for those two values. However, we
   * generate random values for the other data (temperature, current, etc...)
   *
   * @param joint_name The name of the joint, as specified in joints_map.
   * @return The information regarding this joint.
   */
  virtual JointData getJointData(std::string joint_name);
  virtual JointsMap getAllJointsData();

  virtual short setContrl(std::string contrlr_name, JointControllerData ctrlr_data);
  virtual JointControllerData getContrl(std::string ctrlr_name);

  virtual short setConfig(std::vector<std::string> myConfig);
  virtual void getConfig(std::string joint_name);

  /**
   * Generates a set of random data to be published by the diagnostic publisher, but keep the position and target as
   * they are ( those are updated by the sendupdate() function).
   * @return A vector containing all the diagnostics for the hand (motor information, etc...)
   */
  virtual std::vector<DiagnosticData> getDiagnostics();
protected:
  /**
   * Initialise a mapping for the joints.
   */
  void initializeMap();
};

}//end namespace

#endif /* VIRTUAL_ARM_H_ */
