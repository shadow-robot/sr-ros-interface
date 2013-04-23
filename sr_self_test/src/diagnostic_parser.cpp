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
    diagnostics_.push_back( new RTLoopDiagnostics("Realtime Control Loop"));
    diagnostics_.push_back( new EtherCATMasterDiagnostics("EtherCAT Master"));
    diagnostics_.push_back( new MotorDiagnostics("SRDMotor"));
    diagnostics_.push_back( new IsOKDiagnostics("EtherCAT Dual CAN Palm"));
    diagnostics_.push_back( new IsOKDiagnostics("SRBridge")); //TODO: not sure what's the 00

    diag_sub_ = nh_.subscribe("diagnostics_agg", 1, &DiagnosticParser::diagnostics_agg_cb_, this);

    run_tests_();
  }

  void DiagnosticParser::run_tests_()
  {
    //wait for 5 seconds while we parse the diagnostics.
    // spin to make sure we get the messages
    for(size_t i=0; i<50; ++i)
    {
      ros::Duration(0.1).sleep();
      ros::spinOnce();
    }

    BOOST_FOREACH(DiagnosticsMap::value_type diag, all_diagnostics_)
    {
      current_res_ = diag.second->to_string();
      test_runner_->add(diag.first, this, &DiagnosticParser::parse_diagnostics);
    }
  }
  void DiagnosticParser::parse_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if( current_res_.first )
      status.summary( diagnostic_msgs::DiagnosticStatus::OK, current_res_.second );
    else
      status.summary( diagnostic_msgs::DiagnosticStatus::ERROR, current_res_.second );
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

