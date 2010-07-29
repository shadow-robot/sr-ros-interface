#ifndef THREE_DIMENSIONS_H_
#define THREE_DIMENSIONS_H_

#include "optical_dataglove/data_analyser.h"

namespace opticaldataglove{

class ThreeDimensionsAnalyser : public DataAnalyser
{
public :

    ThreeDimensionsAnalyser();

    ~ThreeDimensionsAnalyser();


    virtual std::map<std::string, double> getFingerTipPositions();
    virtual std::map<std::string, double> getHandModel();

};
}

#endif //THREE_DIMENSIONS_H_
