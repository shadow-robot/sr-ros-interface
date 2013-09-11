/**
 * @file   test_self_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Feb  7 09:57:59 2013
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
 * @brief  Testing the self test library using gtest
 *
 *
 */

#include "sr_self_test/sr_self_test.hpp"
#include <ros/ros.h>

#include <diagnostic_msgs/SelfTest.h>
#include <sr_robot_msgs/ManualSelfTest.h>

class MyNode
{
public:

  // self_test::TestRunner is the handles sequencing driver self-tests.
  shadow_robot::SrTestRunner self_test_;

  ros::NodeHandle nh_;

  // A value showing statefulness of tests
  double some_val;

  MyNode() : self_test_()
  {
    self_test_.setID("12345");

    std::vector<std::string> services_to_test;
    services_to_test.push_back("sr_self_test_test/self_test");
    services_to_test.push_back("/rosout/get_loggers");

    self_test_.addServicesTest(services_to_test);

    self_test_.add("Testing plot - not saving", this, &MyNode::test_plot);
    self_test_.add("Testing plot - saving", this, &MyNode::test_plot_save);
    self_test_.add_diagnostic_parser();
    self_test_.addManualTests();
    self_test_.addSensorNoiseTest();
  }

  void test_plot(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    self_test_.plot(get_fake_joints(), true);

    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "A plot should be displayed in a gnuplot window.");
  }

  void test_plot_save(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    std::string path = "/tmp/plot.png";
    self_test_.plot(get_fake_joints(), path, true);

    ros::Duration(0.5).sleep();

    std::ifstream test_file(path.c_str());
    if( test_file.good() )
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "The plot seems to have been saved in " + path);
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No file found at " + path + " the plot was probably not saved correctly.");
  }

  std::map<std::string, std::vector<double> > get_fake_joints()
  {
    std::map<std::string, std::vector<double> > joints;

    std::vector<double> ffj0_pos, ffj0_tar, ffj3_pos, ffj3_tar;
    for (int i = 0; i < 20; ++i)
    {
      ffj0_pos.push_back(1.0/float(i));
      ffj0_tar.push_back(1.0/float(i+1));
      ffj3_pos.push_back(1.0/float(i*2));
      ffj3_tar.push_back(2.0/float(i));
    }

    joints["FFJ0 positions"] = ffj0_pos;
    joints["FFJ0 targets"] = ffj0_tar;
    joints["FFJ3 positions"] = ffj3_pos;
    joints["FFJ3 targets"] = ffj3_tar;

    return joints;
  }

  bool spin()
  {
    while (nh_.ok())
    {
      ros::Duration(0.01).sleep();

      self_test_.checkTest();

      ros::spinOnce();
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sr_self_test_test");

  MyNode n;
  n.spin();

  return(0);
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
