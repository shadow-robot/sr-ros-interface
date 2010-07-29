#ifndef JOINT_DATA_H_
#define JOINT_DATA_H_

#include <string>

namespace opticaldataglove{

class JointData
{
public:
  double position;
  double target;
  double temperature;
  double current;
  double force;
  std::string flags;
  int jointIndex;
  double min;
  double max;
  short isJointZero;

  JointData() :
    position(0.0), target(0.0), temperature(0.0), current(0.0), force(0.0), flags(""), jointIndex(0),
    min(0.0), max(90.0), isJointZero(0)
  {
  }

  JointData(JointData& jd) :
    position(jd.position), target(jd.target), temperature(jd.temperature), current(jd.current), force(jd.force),
        flags(jd.flags), jointIndex(jd.jointIndex), min(jd.min), max(jd.max), isJointZero(jd.isJointZero)
  {
  }

  JointData(const JointData& jd) :
    position(jd.position), target(jd.target), temperature(jd.temperature), current(jd.current), force(jd.force),
        flags(jd.flags), jointIndex(jd.jointIndex), min(jd.min), max(jd.max), isJointZero(jd.isJointZero)
  {
  }
};

}
#endif //JOINT_DATA_H_
