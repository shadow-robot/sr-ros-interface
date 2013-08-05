/**
 * @file   biotac_transmission.hpp
 * @author Norman Hendrich, Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Jul 17 2013, Tue Jun 28 11:35:05 2011
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

#ifndef _BIOTAC_TRANSMISSION_HPP_
#define _BIOTAC_TRANSMISSION_HPP_

#include <tinyxml.h>
#include "pr2_mechanism_model/transmission.h"
#include "pr2_mechanism_model/joint.h"
#include "pr2_hardware_interface/hardware_interface.h"
#include "pr2_mechanism_model/joint_calibration_simulator.h"

namespace sr_mechanism_model
{
  class BiotacTransmission : public pr2_mechanism_model::Transmission
  {
  public:
    BiotacTransmission() {}
    ~BiotacTransmission() {}

    bool initXml(TiXmlElement *config, pr2_mechanism_model::Robot *robot);
    bool initXml(TiXmlElement *config);

    double mechanical_reduction_;

    void propagatePosition(std::vector<pr2_hardware_interface::Actuator*>&,
                           std::vector<pr2_mechanism_model::JointState*>&);
    void propagatePositionBackwards(std::vector<pr2_mechanism_model::JointState*>&,
                                    std::vector<pr2_hardware_interface::Actuator*>&);
    void propagateEffort(std::vector<pr2_mechanism_model::JointState*>&,
                         std::vector<pr2_hardware_interface::Actuator*>&);
    void propagateEffortBackwards(std::vector<pr2_hardware_interface::Actuator*>&,
                                  std::vector<pr2_mechanism_model::JointState*>&);

  private:
    int simulated_actuator_timestamp_initialized_;
    ros::Time simulated_actuator_start_time_;

    pr2_mechanism_model::JointCalibrationSimulator joint_calibration_simulator_;

    bool init_joint(TiXmlElement *jel, pr2_mechanism_model::Robot *robot);

    static const double BIOTAC_ANGLE;
  };

} //end namespace sr_mechanism_model

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
