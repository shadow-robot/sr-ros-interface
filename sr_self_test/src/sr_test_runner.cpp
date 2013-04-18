/**
 * @file   sr_test_runner.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Feb 4, 2013
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
 * @brief This class inherits from the default ROS Test Runner. It adds a set of useful
 *        functionalities for testing topics and services more easily.
 *
 *
 */

#include "sr_self_test/sr_test_runner.hpp"

namespace shadow_robot
{
//const double SrTestRunner::SERVICE_TIMEOUT_CONST_ = 1.0;

  SrTestRunner::SrTestRunner() :
    self_test::TestRunner(), index_service_to_test_(0)
  {
  };

  SrTestRunner::~SrTestRunner()
  {
  };

  void SrTestRunner::addTopicTest(std::string topic_name, double frequency)
  {
  };

  void SrTestRunner::addServicesTest(std::vector<std::string> services_to_test)
  {
    services_to_test_ = services_to_test;
    index_service_to_test_=0;

    for (size_t i=0; i < services_to_test_.size(); ++i)
    {
      add("Testing "+services_to_test_[i]+" is present.", this,  &SrTestRunner::service_test_cb_);
    }
  };

  void SrTestRunner::service_test_cb_(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if( ros::service::exists(services_to_test_[index_service_to_test_], false) )
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Service "+services_to_test_[index_service_to_test_]+" exists.");
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Service "+services_to_test_[index_service_to_test_]+" not available.");

    if(index_service_to_test_ + 1 < services_to_test_.size())
      index_service_to_test_ ++;
  };

  void SrTestRunner::plot(std::map<std::string, std::vector<double> > joints)
  {
    plot(joints, "");
  }

  void SrTestRunner::plot(std::map<std::string, std::vector<double> > joints, bool testing)
  {
    plot(joints, "", testing);
  }

  void SrTestRunner::plot(std::map<std::string, std::vector<double> > joints, std::string path)
  {
    plot(joints, path, false);
  }

  void SrTestRunner::plot(std::map<std::string, std::vector<double> > joints, std::string path, bool testing)
  {
    if( testing )
      gnuplot_.reset(new Gnuplot("gnuplot"));//close the window right after the test when running a test
    else
      gnuplot_.reset(new Gnuplot("gnuplot -persist"));

    //saving the plot to file if path provided
    if( path != "" )
    {
      *gnuplot_.get() << "set terminal png\n";
      *gnuplot_.get() << "set output '"+path+"'\n";
    }

    //plot legend and style
    std::string cmd = "plot ";
    std::string title = "";
    std::map<std::string, std::vector<double> >::const_iterator last_it = joints.end();
    --last_it;
    for (std::map<std::string, std::vector<double> >::const_iterator it = joints.begin(); it != joints.end(); ++it)
    {
      cmd += " '-' with lines title '"+it->first+"'";
      if( it == last_it)
        cmd += "\n";
      else
        cmd += ",";

      title += it->first + " ";
    }

    *gnuplot_.get() << "set title '"+title+"'\n";
    *gnuplot_.get() << cmd;

    //plotting the data
    for (std::map<std::string, std::vector<double> >::const_iterator it = joints.begin(); it != joints.end(); ++it)
    {
      gnuplot_->send(it->second);
    }
  }

  void SrTestRunner::addManualTests()
  {
    manual_tests_.reset(new ManualTests());
    add("Manual Tests", manual_tests_.get(), &ManualTests::run_manual_tests);
  }
} //end namespace


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
