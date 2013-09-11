/**
 * @file   diagnostics_specific.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Apr 23 05:55:49 2013
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
 * @brief Specific diagnostics parsers (for the real time loop diag, the etherCAT
 *        master, etc...)
 *
 *
 */

#ifndef _DIAGNOSTIC_SPECIFIC_HPP_
#define _DIAGNOSTIC_SPECIFIC_HPP_

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/variant.hpp>
#include <sstream>
#include <ros/ros.h>

#include "sr_self_test/diagnostics_parser/diagnostics_common.hpp"

namespace shadow_robot
{
  class RTLoopDiagnostics
    : public MinMaxDiagnostics
  {
  public:
    RTLoopDiagnostics(std::string name, self_test::TestRunner* test_runner)
      : MinMaxDiagnostics(name, test_runner)
    {
      values_.reset(new DiagMap() );
      DiagnosticTest jitter;
      jitter.min_max = std::make_pair(0.0, 100.0); //min and max acceptable jitter
      values_->insert( std::pair<std::string, DiagnosticTest>("Avg Loop Jitter (us)", jitter) );
    }

    ~RTLoopDiagnostics()
    {}

    virtual std::auto_ptr<BaseDiagnostics> shallow_clone(std::string name)
    {
      std::auto_ptr<BaseDiagnostics> tmp( new RTLoopDiagnostics(name, test_runner_) );
      return tmp;
    };

  private:
    double avg_jitter;
    int control_loop_overruns;
  };

  class EtherCATMasterDiagnostics
    : public MinMaxDiagnostics
  {
  public:
    EtherCATMasterDiagnostics(std::string name, self_test::TestRunner* test_runner)
      : MinMaxDiagnostics(name, test_runner)
    {
      values_.reset(new DiagMap() );
      DiagnosticTest dropped_packet;
      dropped_packet.min_max = std::make_pair(0, 200); //min and max acceptable dropped packets
      //max  (TODO: this should be a ratio dropped/sent packets??)
      values_->insert( std::pair<std::string, DiagnosticTest>("Dropped Packets", dropped_packet) );
    }

    ~EtherCATMasterDiagnostics()
    {};

    virtual std::auto_ptr<BaseDiagnostics> shallow_clone(std::string name)
    {
      std::auto_ptr<BaseDiagnostics> tmp( new EtherCATMasterDiagnostics(name, test_runner_) );
      return tmp;
    };
  };


  class MotorDiagnostics
    : public MinMaxDiagnostics
  {
  public:
    MotorDiagnostics(std::string name, self_test::TestRunner* test_runner)
      : MinMaxDiagnostics(name, test_runner)
    {
      values_.reset(new DiagMap() );
      DiagnosticTest voltage;
      voltage.min_max = std::make_pair(23.5, 24.5); //min and max acceptable voltage
      values_->insert( std::pair<std::string, DiagnosticTest>("Measured Voltage", voltage) );

      DiagnosticTest temperature;
      temperature.min_max = std::make_pair(20.0, 50.0); //min and max acceptable temperature
      values_->insert( std::pair<std::string, DiagnosticTest>("Temperature", temperature) );
    };

    ~MotorDiagnostics() {};

    virtual std::auto_ptr<BaseDiagnostics> shallow_clone(std::string name)
    {
      std::auto_ptr<BaseDiagnostics> tmp( new MotorDiagnostics(name, test_runner_) );
      return tmp;
    };
  };
}

  /* For the emacs weenies in the crowd.
     Local Variables:
     c-basic-offset: 2
     End:
  */

#endif /* _DIAGNOSTIC_SPECIFIC_HPP_ */
