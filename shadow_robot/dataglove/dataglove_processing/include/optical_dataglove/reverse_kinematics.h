#ifndef REVERSE_KINEMATICS_H_
#define REVERSE_KINEMATICS_H_

#include "optical_dataglove/position_mapper.h"

namespace opticaldataglove{

/*
 * Class to use reverse kinematics to perform the mapping between fingertips position and joint positions
 */
class ReverseKinematics : public PositionMapper
{
public :

/*
 * Constructor
 * @param data_analyser : DataAnalyser which is getting the fingertips positions
 */
    ReverseKinematics(boost::shared_ptr<DataAnalyser> data_analyser);
    
    ~ReverseKinematics();

    virtual std::map<std::string, JointData> getHandPositions();

private : 
    
    virtual void initializeHandPositions();

};
}

#endif //REVERSE_KINEMATICS_H_
