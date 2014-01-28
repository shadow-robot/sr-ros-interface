/**
 * @file virtual_shadowhand_library.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 10 Nov 2010
 *
 * @brief 
 *
 *
 */

#ifndef VIRTUAL_SHADOWHAND_LIBRARY_H_
#define VIRTUAL_SHADOWHAND_LIBRARY_H_

#include "sr_hand/hand/sr_articulated_robot.h"

namespace shadowrobot
{

class VirtualShadowhandLibrary : public virtual SRArticulatedRobot
{
public:
    VirtualShadowhandLibrary();
    virtual ~VirtualShadowhandLibrary(){};

    virtual short sendupdate( std::string joint_name, double target );
    virtual JointData getJointData( std::string joint_name );
    virtual JointsMap getAllJointsData();

    virtual short setContrl( std::string contrlr_name, JointControllerData ctrlr_data );
    virtual JointControllerData getContrl( std::string ctrlr_name );

    virtual short setConfig( std::vector<std::string> myConfig );
    virtual void getConfig( std::string joint_name );
    virtual std::vector<DiagnosticData> getDiagnostics();
};//end class

}//end namespace
#endif /* VIRTUAL_SHADOWHAND_LIBRARY_H_ */
