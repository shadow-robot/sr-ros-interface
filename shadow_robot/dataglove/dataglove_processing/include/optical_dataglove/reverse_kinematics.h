#ifndef REVERSE_KINEMATICS_H_
#define REVERSE_KINEMATICS_H_

#include "optical_dataglove/position_mapper.h"

namespace opticaldataglove{

class ReverseKinematics : public PositionMapper
{
public :

    ReverseKinematics(boost::shared_ptr<DataAnalyser> data_analyser);
    
    ~ReverseKinematics();

    virtual std::map<std::string, JointData> getHandPositions();

private : 
    
    virtual void initializeHandPositions();

};
}

#endif //REVERSE_KINEMATICS_H_
