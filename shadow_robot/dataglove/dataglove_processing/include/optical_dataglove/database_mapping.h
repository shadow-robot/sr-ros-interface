#ifndef DATABASEMAPPING_H_
#define DATABASEMAPPING_H_

#include "optical_dataglove/position_mapper.h"

namespace opticaldataglove{

/*
 * Class used to perform the conversion between fingertips positions to joints values using a Database of known positions
 */
class DatabaseMapping : public PositionMapper
{
public :

/*
 * Constructor
 * @param data_analyser : data analyser performing computing to get information from hand
 */
    DatabaseMapping(boost::shared_ptr<DataAnalyser> data_analyser);
    
/*
 * Destructor
 */
    ~DatabaseMapping();

/*
 * Returns a map with the angles of that the hand should take
 */
    virtual std::map<std::string, JointData> getHandPositions();

private :
    
/*
 * Initializes the hand position
 */
   virtual void initializeHandPositions(); 

};
}

#endif //#DATABASEMAPPING_H_
