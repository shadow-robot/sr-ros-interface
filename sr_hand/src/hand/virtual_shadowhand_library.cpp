/**
 * @file virtual_shadowhand_library.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 10 Nov 2010
 *
 * @brief 
 *
 *
 */

#include "sr_hand/hand/virtual_shadowhand_library.h"
#include <ros/ros.h>
#include <map>

namespace shadowrobot
{

VirtualShadowhandLibrary::VirtualShadowhandLibrary() :
    SRArticulatedRobot()
{
    joints_map_mutex.lock();

     JointData tmpData;
     JointData tmpDataZero;
     JointControllerData tmpController;
     tmpDataZero.isJointZero = 1;
     tmpDataZero.max = 180.0;

     joints_map["FFJ0"] = tmpDataZero;
     joints_map["FFJ1"] = tmpData;
     joints_map["FFJ2"] = tmpData;
     joints_map["FFJ3"] = tmpData;
     tmpData.min = -20.0;
     tmpData.max = 20.0;
     joints_map["FFJ4"] = tmpData;

     joints_map["MFJ0"] = tmpDataZero;
     tmpData.min = 0.0;
     tmpData.max = 90.0;
     joints_map["MFJ1"] = tmpData;
     joints_map["MFJ2"] = tmpData;
     joints_map["MFJ3"] = tmpData;
     tmpData.min = -20.0;
     tmpData.max = 20.0;
     joints_map["MFJ4"] = tmpData;

     joints_map["RFJ0"] = tmpDataZero;
     tmpData.min = 0.0;
     tmpData.max = 90.0;
     joints_map["RFJ1"] = tmpData;
     joints_map["RFJ2"] = tmpData;
     joints_map["RFJ3"] = tmpData;
     tmpData.min = -20.0;
     tmpData.max = 20.0;
     joints_map["RFJ4"] = tmpData;

     joints_map["LFJ0"] = tmpDataZero;
     tmpData.min = 0.0;
     tmpData.max = 90.0;
     joints_map["LFJ1"] = tmpData;
     joints_map["LFJ2"] = tmpData;
     joints_map["LFJ3"] = tmpData;
     tmpData.min = -20.0;
     tmpData.max = 20.0;
     joints_map["LFJ4"] = tmpData;
     tmpData.min = 0.0;
     tmpData.max = 45.0;
     joints_map["LFJ5"] = tmpData;

     tmpData.min = 0.0;
     tmpData.max = 90.0;
     joints_map["THJ1"] = tmpData;
     tmpData.min = -40.0;
     tmpData.max = 40.0;
     joints_map["THJ2"] = tmpData;
     tmpData.min = -15.0;
     tmpData.max = 15.0;
     joints_map["THJ3"] = tmpData;
     tmpData.min = 0.0;
     tmpData.max = 75.0;
     joints_map["THJ4"] = tmpData;
     tmpData.min = -60.0;
     tmpData.max = 60.0;
     joints_map["THJ5"] = tmpData;

     tmpData.min = -30.0;
     tmpData.max = 45.0;
     joints_map["WRJ1"] = tmpData;
     tmpData.min = -30.0;
     tmpData.max = 10.0;
     joints_map["WRJ2"] = tmpData;

     joints_map_mutex.unlock();
}

short VirtualShadowhandLibrary::sendupdate( std::string joint_name, double target )
{
    return (short)0;
}

JointData VirtualShadowhandLibrary::getJointData( std::string joint_name )
{
    JointData tmp;
    return tmp;
}
std::map<std::string, JointData> VirtualShadowhandLibrary::getAllJointsData()
{
    joints_map_mutex.lock();
    JointsMap tmp_map = JointsMap(joints_map);
    joints_map_mutex.unlock();
    return tmp_map;
}
short VirtualShadowhandLibrary::setContrl( std::string contrlr_name, JointControllerData ctrlr_data )
{
    return (short)0;
}
JointControllerData VirtualShadowhandLibrary::getContrl( std::string ctrlr_name )
{
    JointControllerData tmp;
    return tmp;
}
short VirtualShadowhandLibrary::setConfig( std::vector<std::string> myConfig )
{
    return (short)0;
}
void VirtualShadowhandLibrary::getConfig( std::string joint_name )
{
}
std::vector<DiagnosticData> VirtualShadowhandLibrary::getDiagnostics()
{
    std::vector<DiagnosticData> tmp;
    return tmp;
}
}//end namespace
