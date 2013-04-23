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

#include "sr_self_test/diagnostics_parser/motor_test.hpp"

namespace shadow_robot
{
  MotorDiagnostics::MotorDiagnostics(std::string name, self_test::TestRunner* test_runner)
    : MinMaxDiagnostics(name, test_runner)
  {
    values_.reset(new DiagMap() );
    DiagnosticTest voltage;
    voltage.min_max = std::make_pair(23.5, 24.5); //min and max acceptable voltage
    values_->insert( std::pair<std::string, DiagnosticTest>("Measured Voltage", voltage) );

    DiagnosticTest temperature;
    temperature.min_max = std::make_pair(20.0, 50.0); //min and max acceptable temperature
    values_->insert( std::pair<std::string, DiagnosticTest>("Temperature", temperature) );
  }

  MotorDiagnostics::~MotorDiagnostics()
  {};

  std::auto_ptr<BaseDiagnostics> MotorDiagnostics::shallow_clone(std::string name)
  {
    std::auto_ptr<BaseDiagnostics> tmp( new MotorDiagnostics(name, test_runner_) );
    return tmp;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

