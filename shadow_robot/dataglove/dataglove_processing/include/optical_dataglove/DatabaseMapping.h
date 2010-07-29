#ifndef DATABASEMAPPING_H_
#define DATABASEMAPPING_H_

#include "optical_dataglove/position_mapper.h"

namespace opticaldataglove{

class DatabaseMapping : public PositionMapper
{
public :

    DatabaseMapping()
    {
    };
    
    ~DatabaseMapping()
    {
    };

    std::map<std::string, JointData> getHandPositions();

};
}

#endif //#DATABASEMAPPING_H_
