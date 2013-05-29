/**
 * @file   sensor_noise_test.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Apr 18 06:33:35 2013
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
 * @brief We use this class to parse the joint states while the hand is immobile
 *        and check the noise of the sensor.
 *
 *
 */

#include "sr_self_test/sensor_noise_test.hpp"

namespace shadow_robot
{
  const double SensorNoiseTest::MAX_NOISE_CONST_ = 0.0087; //0.5 degrees
  const double SensorNoiseTest::NOISE_EPSILON_CONST_ = 0.000000001; //So small that no real sensor can have smaller noise

  SensorNoiseTest::SensorNoiseTest()
  {
  }

  void SensorNoiseTest::test_sensor_noise(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // We subscribe to the joint_states topic when the test starts running.
    // This way we avoid the initial ramping up of the filtered position when the driver starts
    // (first pos=0, then the real sensor data is fed into the filter, and the position starts approaching that value)
    // We can perfectly assume that the filtered value is stabilized here.
    joint_states_sub_ = nh_.subscribe("joint_states", 50, &SensorNoiseTest::joint_states_cb_, this);

    ros::Rate rate(100);
    for (size_t i = 0; i < 400; ++i)
    {
      ros::spinOnce();
      rate.sleep();
    }

    std::vector<std::string> failed_joints;
    std::vector<double> failed_noises;
    double min, max;
    std::map<std::string, std::vector<double> >::iterator joint_states;
    for(joint_states = all_joint_states_.begin(); joint_states != all_joint_states_.end(); ++joint_states)
    {
      if(joint_states->second.size() > 1 )
      {
        min = joint_states->second[0];
        max = joint_states->second[0];

        //starting loop at 1, as we already assigned 0 to min/max
        for (size_t i = 1; i < joint_states->second.size(); ++i)
        {
          if(joint_states->second[i] < min)
            min = joint_states->second[i];
          if(joint_states->second[i] > max)
            max = joint_states->second[i];
        }

        double noise = max - min;
        ROS_DEBUG("Joint %s Pos min: %lf  max: %lf  noise: %lf", joint_states->first.c_str(),  min , max , noise );
        
        if( noise > MAX_NOISE_CONST_ || noise <= NOISE_EPSILON_CONST_ )
        {
          failed_joints.push_back(joint_states->first);
          failed_noises.push_back( noise );
        }
      }
    }

    if( failed_joints.size() > 0 )
    {
      std::stringstream ss;
      ss << "Noises out of threshold: ";
      for (size_t i = 0; i < failed_joints.size(); ++i)
      {
        ss << "[" << failed_joints[i] << " -> " << failed_noises[i] << " ] ";
      }
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR, ss.str());
    }
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::OK, "All the noises are below threshold.");

    joint_states_sub_.shutdown();
  }

  void SensorNoiseTest::joint_states_cb_(const sensor_msgs::JointState::ConstPtr& msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      all_joint_states_[ msg->name[i] ].push_back( msg->position[i] );
    }
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

