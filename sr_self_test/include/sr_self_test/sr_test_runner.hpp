/**
 * @file   sr_test_runner.hpp
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

#ifndef SR_TEST_RUNNER_HPP_
#define SR_TEST_RUNNER_HPP_

#include <self_test/self_test.h>
#include "sr_self_test/gnuplot-iostream.h"
#include "sr_self_test/diagnostic_parser.hpp"
#include "sr_self_test/manual_test.hpp"

#include "sr_self_test/sensor_noise_test.hpp"

namespace shadow_robot
{
class SrTestRunner : public self_test::TestRunner
{
public:
  SrTestRunner();

  virtual ~SrTestRunner();

  using DiagnosticTaskVector::add;
  using TestRunner::setID;

  void addTopicTest(std::string topic_name, double frequency);
  void addServicesTest(std::vector<std::string> services_to_test);
  ///Those tests require the user input
  void addManualTests();

  ///Tests the noise of the pose sensor
  void addSensorNoiseTest();

  void plot(std::map<std::string, std::vector<double> > joints);
  void plot(std::map<std::string, std::vector<double> > joints, bool testing);
  void plot(std::map<std::string, std::vector<double> > joints, std::string path);
  void plot(std::map<std::string, std::vector<double> > joints, std::string path, bool testing);

  ///Adding a test which parses diagnostics for jitter, dropped messages, etc...
  void add_diagnostic_parser();

private:
  static const double SERVICE_TIMEOUT_CONST_;

  std::vector<std::string> services_to_test_;
  void service_test_cb_(diagnostic_updater::DiagnosticStatusWrapper& status);
  size_t index_service_to_test_;

  boost::shared_ptr<Gnuplot> gnuplot_;

  ///Class used for parsing the diagnostics
  boost::shared_ptr<DiagnosticParser> diagnostic_parser_;

  ///runs manual test (visual calibration check, tactiles...)
  boost::shared_ptr<ManualTests> manual_tests_;

  boost::shared_ptr<SensorNoiseTest> sensor_noise_test_;
};

}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* SR_TEST_RUNNER_HPP_ */
