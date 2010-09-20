#include  "optical_dataglove/database_mapping.h"

namespace opticaldataglove{

DatabaseMapping::DatabaseMapping(boost::shared_ptr<DataAnalyser> data_analyser): PositionMapper()
{
    this->dataAnalyser=data_analyser;
    initializeHandPositions();
}

DatabaseMapping::~DatabaseMapping()
{

}

std::map<std::string, JointData> DatabaseMapping::getHandPositions()
{
    return handPositions;
}

void DatabaseMapping::initializeHandPositions()
{
    this->handPositions["THJ1"] = JointData();
    this->handPositions["THJ1"].position =0.0;
    this->handPositions["THJ2"] = JointData();
    this->handPositions["THJ2"].position = 0.0;
    this->handPositions["THJ3"] = JointData();
    this->handPositions["THJ3"].position = 0.0;
    this->handPositions["THJ4"] = JointData();
    this->handPositions["THJ4"].position = 0.0;
    this->handPositions["THJ5"] = JointData();
    this->handPositions["THJ5"].position = 0.0;
    this->handPositions["FFJ0"] = JointData();
    this->handPositions["FFJ0"].position = 0.0;
    this->handPositions["FFJ3"] = JointData();
    this->handPositions["FFJ3"].position = 0.0;
    this->handPositions["FFJ4"] = JointData();
    this->handPositions["FFJ4"].position = 0.0;
    this->handPositions["MFJ0"] = JointData();
    this->handPositions["MFJ0"].position = 0.0;
    this->handPositions["MFJ3"] = JointData();
    this->handPositions["MFJ3"].position = 0.0;
    this->handPositions["MFJ4"] = JointData();
    this->handPositions["MFJ4"].position = 0.0;
    this->handPositions["RFJ0"] = JointData();
    this->handPositions["RFJ0"].position = 0.0;
    this->handPositions["RFJ3"] = JointData();
    this->handPositions["RFJ3"].position = 0.0;
    this->handPositions["RFJ4"] = JointData();
    this->handPositions["RFJ4"].position = 0.0;
    this->handPositions["LFJ0"] = JointData();
    this->handPositions["LFJ0"].position = 0.0;
    this->handPositions["LFJ3"] = JointData();
    this->handPositions["LFJ3"].position = 0.0;
    this->handPositions["LFJ4"] = JointData();
    this->handPositions["LFJ4"].position = 0.0;
    this->handPositions["LFJ5"] = JointData();
    this->handPositions["LFJ5"].position = 0.0;
 
}

}
