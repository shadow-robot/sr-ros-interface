#ifndef THREE_DIMENSIONS_H_
#define THREE_DIMENSIONS_H_

#include "optical_dataglove/data_analyser.h"

namespace opticaldataglove{

/*
 * Class performing the conversion between 3D pictures to fingertips positions
 */
class ThreeDimensionsAnalyser : public DataAnalyser
{
public :

/*
 * Constructor
 */
    ThreeDimensionsAnalyser();

/*
 * Destructor
 */
    ~ThreeDimensionsAnalyser();


    virtual std::map<std::string, double> getFingerTipPositions();
    virtual std::map<std::string, double> getHandModel();

};
}

#endif //THREE_DIMENSIONS_H_
