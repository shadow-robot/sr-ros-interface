#ifndef POSITION_MAPPER_H_
#define POSITION_MAPPER_H_

#include "optical_dataglove/joint_data.h"
#include "optical_dataglove/data_analyser.h"
#include <boost/smart_ptr.hpp>
#include <map>

namespace opticaldataglove{

class PositionMapper
{

public:
    
    PositionMapper()
    {
    };

    ~PositionMapper()
    {
    };

    virtual std::map<std::string, JointData> getHandPositions()=0;

protected:
    
    std::map<std::string, JointData> handPositions;
    std::map<std::string, double> fingerTipPositions;
    boost::shared_ptr<DataAnalyser> dataAnalyser;
    virtual void initializeHandPositions()=0;
};
}

#endif //POSITION_MAPPER_H_
