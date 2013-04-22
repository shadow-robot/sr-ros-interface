/**
 * @file   motor_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Apr 22 05:47:43 2013
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
 *
 * @brief Run different tests on the motors (parses diagnostics, apply PWM offset
 *        and measure current, test strain gauges, etc.. )
 *
 *
 */

#include "sr_self_test/motor_test.hpp"

namespace shadow_robot
{
  MotorDiagnostics::MotorDiagnostics(std::string name)
    : MinMaxDiagnostics(name)
  {
    values_.reset(new DiagMap() );
    std::pair<std::vector<DiagValues>, std::vector<DiagValues> > voltage;
    voltage.second.resize(2);
    voltage.second[0] = 23.5; //min
    voltage.second[1] = 24.5; //max
    values_->insert( std::pair<std::string, std::pair<std::vector<DiagValues>, std::vector<DiagValues> > >("Measured Voltage", voltage) );

    std::pair<std::vector<DiagValues>, std::vector<DiagValues> > temperature;
    temperature.second.resize(2);
    temperature.second[0] = 20.0; //min
    temperature.second[1] = 50.0; //max
    values_->insert( std::pair<std::string, std::pair<std::vector<DiagValues>, std::vector<DiagValues> > >("Temperature", temperature) );
  }

  MotorDiagnostics::~MotorDiagnostics()
  {};

  std::auto_ptr<BaseDiagnostics> MotorDiagnostics::shallow_clone(std::string name)
  {
    std::auto_ptr<BaseDiagnostics> tmp( new MotorDiagnostics(name) );
    return tmp;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

