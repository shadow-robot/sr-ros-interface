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
  gnuplot_.reset(new Gnuplot("gnuplot -persist"));
  plot_();
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

void SrTestRunner::plot_()
{
	double arr[] = { 1, 3, 2 };

	*gnuplot_.get() << "plot '-' with lines\n";
	gnuplot_->send(arr);
}
} //end namespace


