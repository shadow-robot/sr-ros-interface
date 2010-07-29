#ifndef TWO_DIMENSIONS_H_
#define TWO_DIMENSIONS_H_

#include "optical_dataglove/data_analyser.h"

namespace opticaldataglove{

class TwoDimensionsAnalyser : public DataAnalyser
{
public :

    TwoDimensionsAnalyser();

    ~TwoDimensionsAnalyser();


    virtual std::map<std::string, double> getFingerTipPositions();
    virtual std::map<std::string, double> getHandModel();

};
}

#endif //TWO_DIMENSIONS_H_
