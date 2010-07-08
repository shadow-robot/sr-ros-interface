/**
* @file   virtual_shadowhand.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Tue May 25 17:50:42 2010
*
* @brief
*
*
*/

#include "sr_hand/hand/virtual_shadowhand.h"

#include <time.h>
#include <ros/ros.h>

namespace shadowhand
{
VirtualShadowhand::VirtualShadowhand()
: Shadowhand()
{
  srand ( time(NULL) );
  initializeMap();
}

VirtualShadowhand::~VirtualShadowhand()
{
}

void VirtualShadowhand::initializeMap()
{
  JointData tmpData;
  JointData tmpDataZero;
  JointControllerData tmpController;
  tmpDataZero.isJointZero = 1;
  tmpDataZero.max = 180.0;

  joints_map["FFJ0"] = tmpDataZero;
  controllers_map["FFJ0"] = tmpController;
  joints_map["FFJ1"] = tmpData;
  controllers_map["FFJ1"] = tmpController;
  joints_map["FFJ2"] = tmpData;
  controllers_map["FFJ2"] = tmpController;
  joints_map["FFJ3"] = tmpData;
  controllers_map["FFJ3"] = tmpController;
  tmpData.min = -25.0;
  tmpData.max = 25.0;
  joints_map["FFJ4"] = tmpData;
  controllers_map["FFJ4"] = tmpController;

  joints_map["MFJ0"] = tmpDataZero;
  controllers_map["MFJ0"] = tmpController;
  tmpData.min = 0.0;
  tmpData.max = 90.0;
  joints_map["MFJ1"] = tmpData;
  controllers_map["MFJ1"] = tmpController;
  joints_map["MFJ2"] = tmpData;
  controllers_map["MFJ2"] = tmpController;
  joints_map["MFJ3"] = tmpData;
  controllers_map["MFJ3"] = tmpController;
  tmpData.min = -25.0;
  tmpData.max = 25.0;
  joints_map["MFJ4"] = tmpData;
  controllers_map["MFJ4"] = tmpController;

  joints_map["RFJ0"] = tmpDataZero;
  controllers_map["RFJ0"] = tmpController;
  tmpData.min = 0.0;
  tmpData.max = 90.0;
  joints_map["RFJ1"] = tmpData;
  controllers_map["RFJ1"] = tmpController;
  joints_map["RFJ2"] = tmpData;
  controllers_map["RFJ2"] = tmpController;
  joints_map["RFJ3"] = tmpData;
  controllers_map["RFJ3"] = tmpController;
  tmpData.min = -25.0;
  tmpData.max = 25.0;
  joints_map["RFJ4"] = tmpData;
  controllers_map["RFJ4"] = tmpController;

  joints_map["LFJ0"] = tmpDataZero;
  controllers_map["LFJ0"] = tmpController;
  tmpData.min = 0.0;
  tmpData.max = 90.0;
  joints_map["LFJ1"] = tmpData;
  controllers_map["LFJ1"] = tmpController;
  joints_map["LFJ2"] = tmpData;
  controllers_map["LFJ2"] = tmpController;
  joints_map["LFJ3"] = tmpData;
  controllers_map["LFJ3"] = tmpController;
  tmpData.min = -25.0;
  tmpData.max = 25.0;
  joints_map["LFJ4"] = tmpData;
  controllers_map["LFJ4"] = tmpController;
  tmpData.min = 0.0;
  tmpData.max = 45.0;
  joints_map["LFJ5"] = tmpData;
  controllers_map["LFJ5"] = tmpController;

  tmpData.min = 0.0;
  tmpData.max = 90.0;
  joints_map["THJ1"] = tmpData;
  controllers_map["THJ1"] = tmpController;
  tmpData.min = -30.0;
  tmpData.max = 30.0;
  joints_map["THJ2"] = tmpData;
  controllers_map["THJ2"] = tmpController;
  tmpData.min = -15.0;
  tmpData.max = 15.0;
  joints_map["THJ3"] = tmpData;
  controllers_map["THJ3"] = tmpController;
  tmpData.min = 0.0;
  tmpData.max = 75.0;
  joints_map["THJ4"] = tmpData;
  controllers_map["THJ4"] = tmpController;
  tmpData.min = -60.0;
  tmpData.max = 60.0;
  joints_map["THJ5"] = tmpData;
  controllers_map["THJ5"] = tmpController;

  tmpData.min = -30.0;
  tmpData.max = 40.0;
  joints_map["WRJ1"] = tmpData;
  controllers_map["WRJ1"] = tmpController;
  tmpData.min = -30.0;
  tmpData.max = 10.0;
  joints_map["WRJ2"] = tmpData;
  controllers_map["WRJ2"] = tmpController;



  parameters_map["d"] = PARAM_d;
  parameters_map["i"] = PARAM_i;
  parameters_map["p"] = PARAM_p;
  parameters_map["target"] = PARAM_target;
  parameters_map["sensor"] = PARAM_sensor;

  parameters_map["valve"] = PARAM_valve;
  parameters_map["dead"] = PARAM_deadband;
  parameters_map["deadband"] = PARAM_deadband;
  parameters_map["imax"] = PARAM_imax;
  parameters_map["offset"] = PARAM_output_offset;
  parameters_map["shift"] = PARAM_shift;
  parameters_map["max"] = PARAM_output_max;

  //! the parameters for the motors
  parameters_map["motor_maxforce"] = PARAM_motor_maxforce;
  parameters_map["motor_safeforce"] = PARAM_motor_safeforce;

  parameters_map["force_p"] = PARAM_force_p;
  parameters_map["force_i"] = PARAM_force_i;
  parameters_map["force_d"] = PARAM_force_d;

  parameters_map["force_imax"] = PARAM_force_imax;
  parameters_map["force_out_shift"] = PARAM_force_out_shift;
  parameters_map["force_deadband"] = PARAM_force_deadband;
  parameters_map["force_offset"] = PARAM_force_offset;

  parameters_map["sensor_imax"] = PARAM_sensor_imax;
  parameters_map["sensor_out_shift"] = PARAM_sensor_out_shift;
  parameters_map["sensor_deadband"] = PARAM_sensor_deadband;
  parameters_map["sensor_offset"] = PARAM_sensor_offset;
  parameters_map["max_temp"] = PARAM_max_temperature;
  parameters_map["max_temperature"] = PARAM_max_temperature;
  parameters_map["max_current"] = PARAM_max_current;

}

short VirtualShadowhand::sendupdate(std::string joint_name, double target)
{
  JointsMap::iterator iter = joints_map.find(joint_name);

  //not found
  if(iter == joints_map.end())
  {
    ROS_ERROR("Joint not found");
    return -1;
  }

  //if joint 0, send 1/2 of the target to joint 1 and other half to
  //2;
  if( iter->second.isJointZero == 1 )
  {
    //push target and position to the given target for Joint 0
    JointData tmpData0 = JointData(iter->second);
    if( target < tmpData0.min )
      target = tmpData0.min;
    if( target > tmpData0.max )
      target = tmpData0.max;

    tmpData0.position = target;
    tmpData0.target = target;

    joints_map[joint_name] = tmpData0;

    ++iter;
    JointData tmpData1 = JointData(iter->second);
    tmpData1.position = target/2.0;
    tmpData1.target = target/2.0;

    joints_map[iter->first] = tmpData1;

    ++iter;
    JointData tmpData2= JointData(iter->second);
    tmpData2.position = target/2.0;
    tmpData2.target = target/2.0;

    joints_map[iter->first] = tmpData2;
    return 0;
  }

  //joint found
  JointData tmpData(iter->second);

  if( target < tmpData.min )
    target = tmpData.min;
  if( target > tmpData.max )
    target = tmpData.max;

  tmpData.position = target;
  tmpData.target = target;

  joints_map[joint_name] = tmpData;

  return 0;
}

JointData VirtualShadowhand::getJointData(std::string joint_name)
{
  JointsMap::iterator iter = joints_map.find(joint_name);

  //joint found
  if(iter != joints_map.end())
  {
    //return the position
    iter->second.temperature = ((double)(rand() % 100) / 100.0);
    iter->second.current = ((double)(rand() % 100) / 100.0);
    iter->second.force = ((double)(rand() % 100) / 100.0);
    return iter->second;
  }

  ROS_ERROR("Joint %s not found.", joint_name.c_str());
  JointData noData;
  return noData;
}

Shadowhand::JointsMap VirtualShadowhand::getAllJointsData()
{
  for(JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it)
  {
    JointData tmpData = it->second;
    tmpData.temperature = ((double)(rand() % 100) / 100.0);
    tmpData.current = ((double)(rand() % 100) / 100.0);
    tmpData.force = ((double)(rand() % 100) / 100.0);
    tmpData.jointIndex = 0;
    tmpData.flags = "";

    joints_map[it->first] = tmpData;
  }

  return joints_map;
}

short VirtualShadowhand::setContrl(std::string contrlr_name, JointControllerData ctrlr_data)
{
  ControllersMap::iterator iter = controllers_map.find(contrlr_name);

  //joint found
  if(iter != controllers_map.end())
  {
    controllers_map[iter->first] = ctrlr_data;
  }
  else
    ROS_ERROR("Controller %s not found", contrlr_name.c_str());

  return 0;
}

JointControllerData VirtualShadowhand::getContrl(std::string contrlr_name)
{
  ControllersMap::iterator iter = controllers_map.find(contrlr_name);

  //joint found
  if(iter != controllers_map.end())
    return iter->second;

  ROS_ERROR("Controller %s not found", contrlr_name.c_str() );
  JointControllerData no_result;
  return no_result;
}

short VirtualShadowhand::setConfig(std::vector<std::string> myConfig)
{
  ROS_WARN("The set config function is not implemented in the virtual shadowhand.");
  return 0;
}

void VirtualShadowhand::getConfig(std::string joint_name)
{
  ROS_WARN("The get config function is not implemented in the virtual shadowhand.");
}

std::vector<DiagnosticData> VirtualShadowhand::getDiagnostics()
{
  std::vector<DiagnosticData> returnVect;

  for(JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it)
  {
    DiagnosticData tmpDiag;
    tmpDiag.joint_name = it->first;
    tmpDiag.level = 0;
    tmpDiag.flags = "";
    tmpDiag.target_sensor_num = 0;
    tmpDiag.target = it->second.target;
    tmpDiag.position_sensor_num = 0;
    tmpDiag.position = it-> second.position;

    returnVect.push_back(tmpDiag);
  }

  return returnVect;
}
} //end namespace
