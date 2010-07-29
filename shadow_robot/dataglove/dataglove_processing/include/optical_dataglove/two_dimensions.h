#ifndef TWO_DIMENSIONS_H_
#define TWO_DIMENSIONS_H_

#include "optical_dataglove/data_analyser.h"

namespace opticaldataglove{

/*
 * Class performing the conversion between 2D pictures to fingertips positions
 */
class TwoDimensionsAnalyser : public DataAnalyser
{
public :

/*
 * Constructor
 */
    TwoDimensionsAnalyser();

/*
 * Destructor
 */
    ~TwoDimensionsAnalyser();


    virtual std::map<std::string, double> getFingerTipPositions();
    virtual std::map<std::string, double> getHandModel();

};
}

#endif //TWO_DIMENSIONS_H_
