#ifndef DATABASEMAPPING_H_
#define DATABASEMAPPING_H_

#include "optical_dataglove/position_mapper.h"

namespace opticaldataglove{

class DatabaseMapping : public PositionMapper
{
public :

    DatabaseMapping(boost::shared_ptr<DataAnalyser> data_analyser);
    
    ~DatabaseMapping();

    virtual std::map<std::string, JointData> getHandPositions();

private :
    
   virtual void initializeHandPositions(); 

};
}

#endif //#DATABASEMAPPING_H_
