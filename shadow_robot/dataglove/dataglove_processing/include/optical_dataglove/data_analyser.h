#ifndef DATA_ANALYSER_H_
#define DATA_ANALYSER_H_

#include "optical_dataglove/joint_data.h"
#include <map>

namespace opticaldataglove{

/*
 * Abstract class to define methods to make conversions between the raw source to the fingertips positions
 * Key methods to implement in subclass are getFingerTipPositions and getHandModel, which is used to determine the distances between the human hand articulations
 */
class DataAnalyser
{

public:
/*
 * Constructor
 */
    DataAnalyser()
    {
    };
/*
 * Destructor
 */
    ~DataAnalyser()
    {
    };

    virtual std::map<std::string, double> getFingerTipPositions()=0;
    virtual std::map<std::string, double> getHandModel()=0;

protected:
    
    std::map<std::string, double> fingerTipPositions;
    std::map<std::string, double> handModel;
    

};
}
#endif //DATA_ANALYSER_H_
