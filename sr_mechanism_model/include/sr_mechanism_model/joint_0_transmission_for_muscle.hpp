/**
 * @file   joint_0_transmission_for_muscle.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief This is the implementation of the transmission for the joint 0s.
 * We need a specific transmission which takes into account that 2 joints
 * are actuated with only one actuator.
 *
 *
 */

#ifndef _JOINT_0_TRANSMISSION_FOR_MUSCLE_HPP_
#define _JOINT_0_TRANSMISSION_FOR_MUSCLE_HPP_

#include <tinyxml.h>
#include "ros_ethercat_mechanism_model/transmission.hpp"
#include "ros_ethercat_mechanism_model/joint.hpp"
#include "ros_ethercat_mechanism_model/hardware_interface.hpp"

namespace sr_mechanism_model
{
  class J0TransmissionForMuscle : public ros_ethercat_mechanism_model::Transmission
  {
  public:
    J0TransmissionForMuscle() {}
    ~J0TransmissionForMuscle() {}

    bool initXml(TiXmlElement *config, ros_ethercat_mechanism_model::Robot *robot);

    void propagatePosition(std::vector<ros_ethercat_mechanism_model::Actuator*>&,
                           std::vector<ros_ethercat_mechanism_model::JointState*>&);
    void propagatePositionBackwards(std::vector<ros_ethercat_mechanism_model::JointState*>&,
                                    std::vector<ros_ethercat_mechanism_model::Actuator*>&);
    void propagateEffort(std::vector<ros_ethercat_mechanism_model::JointState*>&,
                         std::vector<ros_ethercat_mechanism_model::Actuator*>&);
    void propagateEffortBackwards(std::vector<ros_ethercat_mechanism_model::Actuator*>&,
                                  std::vector<ros_ethercat_mechanism_model::JointState*>&);

  private:

    bool init_joint(TiXmlElement *jel, ros_ethercat_mechanism_model::Robot *robot);
  };

} //end namespace sr_mechanism_model

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
