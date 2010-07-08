/**
 * @file   virtual_shadowhand.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue May 25 17:51:10 2010
 * 
 * @brief  The Virtual Shadowhand can be used as a simulator. As both the real hand and the virtual hand are children
 * from the shadowhand class, using a virtual or a real hand doesn't change anything in the way you call them in your
 * programs.
 * 
 * 
 */

#ifndef   	VIRTUAL_SHADOWHAND_H_
# define   	VIRTUAL_SHADOWHAND_H_

#include "sr_hand/hand/shadowhand.h"

namespace shadowhand
{
/**
 * @brief  The Virtual Shadowhand can be used as a simulator. As both the real hand and the virtual hand are children
 * from the shadowhand class, using a virtual or a real hand doesn't change anything in the way you call them in your
 * programs.
 *
 */
class VirtualShadowhand : public Shadowhand
{
public:
  /**
   * Initializes the necessary mappings with a static list of names.
   */
  VirtualShadowhand();
  ///destructor
  ~VirtualShadowhand();

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
   * In the virtual hand, getJointData() simply fetches the data from a given joint in the joints_map. As the targets
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
   * Initialize a mapping for the joints as well as a mapping for the controllers.
   */
  void initializeMap();

  ///@see controllers_map
  typedef std::map<std::string, JointControllerData> ControllersMap;
  ///Contains the mapping between the controller names and their data.
  ControllersMap controllers_map;
}; //end class
}
#endif 	    /* !VIRTUAL_SHADOWHAND_H_ */
