/**
 * @file   diagnostic_parser.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Mar 27 06:26:23 2013
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
 * @brief Parsing the diagnostics for jitter, dropped messages, etc...
 *
 *
 */

#include "sr_self_test/diagnostic_parser.hpp"
#include <boost/foreach.hpp>

namespace shadow_robot
{
  DiagnosticParser::DiagnosticParser(self_test::TestRunner* test_runner)
    : test_runner_(test_runner)
  {
    diagnostics_.push_back( new RTLoopDiagnostics("Realtime Control Loop", test_runner_));
    diagnostics_.push_back( new EtherCATMasterDiagnostics("EtherCAT Master", test_runner_));
    diagnostics_.push_back( new MotorDiagnostics("SRDMotor", test_runner_));
    diagnostics_.push_back( new IsOKDiagnostics("EtherCAT Dual CAN Palm", test_runner_));
    diagnostics_.push_back( new IsOKDiagnostics("SRBridge", test_runner_)); //TODO: not sure what's the 00

    run_tests_();
  }

  void DiagnosticParser::run_tests_()
  {
    diag_sub_ = nh_.subscribe("diagnostics_agg", 1, &DiagnosticParser::diagnostics_agg_cb_, this);

    //wait for 5 seconds while we parse the diagnostics.
    // spin to make sure we get the messages
    for(size_t i=0; i<50; ++i)
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }

    BOOST_FOREACH(DiagnosticsMap::value_type diag, all_diagnostics_)
    {
      diag.second->add_test();
    }

    diag_sub_.shutdown();
  }

  void DiagnosticParser::diagnostics_agg_cb_(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
  {
    for( size_t status_i = 0; status_i < msg->status.size(); ++status_i )
    {
      for( size_t diag_i = 0; diag_i < diagnostics_.size() ; ++diag_i )
      {
        if( msg->status[status_i].name.find(diagnostics_[diag_i].name) != std::string::npos )
        {
          std::string full_name = msg->status[status_i].name;
          DiagnosticsMap::iterator it;
          it = all_diagnostics_.find(full_name);

          //insert a new diag if it doesn't exist already
          if( it == all_diagnostics_.end() )
          {
            all_diagnostics_.insert( full_name,
                                     diagnostics_[diag_i].shallow_clone(full_name) );

            it = all_diagnostics_.find(full_name);
          }

          it->second->parse_diagnostics( msg->status[status_i].values,
                                         msg->status[status_i].level,
                                         full_name );
        }
      }
    }
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

