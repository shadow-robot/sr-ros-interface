/**
 * @file   real_shadowhand.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue May 25 17:51:10 2010
 * 
 * @brief The real shadowhand is the ROS interface to Shadow Robot robotic hand.
 * 
 * @todo Make sure it works with the motor hand.
 * 
 */

#ifndef   	REAL_SHADOWHAND_H_
# define   	REAL_SHADOWHAND_H_

#include "sr_hand/hand/shadowhand.h"

namespace shadowhand
{
/**
 * The real shadowhand class is a class used to access the C code of the Dextrous Hand.
 */
class RealShadowhand : public Shadowhand
{
public:
  /**
   * Constructor for the real Shadowhand: initialize the connection to the robot and also initialize the
   * joint_map containing the mapping between joint_names and their data.
   */
  RealShadowhand();

  ///Destructor
  ~RealShadowhand();

  /**
   * Send a new target to a given joint on the robot. The command will be issued to the robot which will move if
   * the controllers are started and tuned. The target is truncated to the correct range.
   *
   * @param joint_name The Joint in joints_map you wish to send the target to.
   * @param target The target in degree
   * @return 0 if success ; -1 if error
   */
  virtual short sendupdate(std::string joint_name, double target);

  virtual JointData getJointData(std::string joint_name);
  virtual JointsMap getAllJointsData();

  virtual short setContrl(std::string contrlr_name, JointControllerData ctrlr_data);
  virtual JointControllerData getContrl(std::string contrlr_name);
  virtual short setConfig(std::vector<std::string> myConfig);
  virtual void getConfig(std::string joint_name);
  virtual std::vector<DiagnosticData> getDiagnostics();
protected:
  /***
   * Initializes the mapping between the joint_names and their data. This function fetches the joint_names from
   * the robot code.
   */
  void initializeMap();
}; //end class
}
#endif 	    /* !REAL_SHADOWHAND_H_ */
