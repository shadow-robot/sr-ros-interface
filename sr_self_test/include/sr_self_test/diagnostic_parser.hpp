/**
 * @file   diagnostic_parser.hpp
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

#ifndef _DIAGNOSTIC_PARSER_HPP_
#define _DIAGNOSTIC_PARSER_HPP_

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <boost/variant.hpp>
#include <sstream>
#include <ros/ros.h>
#include <self_test/self_test.h>

#include "sr_self_test/diagnostics_parser/diagnostics_specific.hpp"

namespace shadow_robot
{
  class DiagnosticParser
  {
  public:
    DiagnosticParser(self_test::TestRunner* test_runner);
    ~DiagnosticParser()
    {};

    void parse_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& status);

  private:
    ros::NodeHandle nh_;

    ///Pointer to the test runner to be able to add new tests for each parser.
    self_test::TestRunner* test_runner_;

    ///ROS subscriber to the diagnostics_agg topic
    ros::Subscriber diag_sub_;

    ///Wait for the diagnostics to be received then run all tests on them
    void run_tests_();

    /**
     * Susbscribed to the diagnostics_agg topic.
     * @param msg new incoming msg
     */
    void diagnostics_agg_cb_(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);

    boost::ptr_vector<BaseDiagnostics> diagnostics_;

    typedef boost::ptr_map<std::string, BaseDiagnostics> DiagnosticsMap;
    DiagnosticsMap all_diagnostics_;
  };
}

  /* For the emacs weenies in the crowd.
     Local Variables:
     c-basic-offset: 2
     End:
  */

#endif /* _DIAGNOSTIC_PARSER_HPP_ */
