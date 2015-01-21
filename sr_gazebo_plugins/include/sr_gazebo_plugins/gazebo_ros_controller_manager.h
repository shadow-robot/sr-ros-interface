/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GAZEBO_CONTROLLER_MANAGER_H
#define GAZEBO_CONTROLLER_MANAGER_H

#include <vector>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>

#include "ros_ethercat_model/hardware_interface.hpp"
#include "ros_ethercat_model/ros_ethercat.hpp"
#include "controller_manager/controller_manager.h"
#include "ros_ethercat_model/robot_state.hpp"
#include <tinyxml.h>
#include <ros/ros.h>
#undef USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#endif

#include "boost/thread/mutex.hpp"

#include <sr_self_test/sr_self_test.hpp>

namespace gazebo
{
class GazeboRosControllerManager : public ModelPlugin
{
public:
  GazeboRosControllerManager();
  virtual ~GazeboRosControllerManager();
  void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

protected:
  // Inherited from gazebo::Controller
  virtual void UpdateChild();

  virtual void ResetChild();

  boost::shared_ptr<shadow_robot::SrSelfTest> self_test_;
private:

  gazebo::physics::ModelPtr parent_model_;
  RosEthercat *state_;
  controller_manager::ControllerManager *cm_;

  /// The fake state helps Gazebo run the transmissions backwards, so
  /// that it can figure out what its joints should do based on the
  /// actuator values.
  ros_ethercat_model::RobotState *fake_state_;
  std::vector<gazebo::physics::JointPtr>  joints_;

  /*
   * \brief read pr2.xml for actuators, and pass tinyxml node to mechanism control node's initXml.
   */
  void ReadPr2Xml();

  /*
   *  \brief pointer to ros node
   */
  ros::NodeHandle* rosnode_;

  ///\brief ros service callback
  /*
   *  \brief tmp vars for performance checking
   */
  double wall_start_, sim_start_;

  /// \brief set topic name of robot description parameter
  std::string robotParam;
  std::string robotNamespace;

  bool fake_calibration_;


#ifdef USE_CBQ
  private: ros::CallbackQueue controller_manager_queue_;
  private: void ControllerManagerQueueThread();
  private: boost::thread controller_manager_callback_queue_thread_;
#endif
  void ControllerManagerROSThread();
  boost::thread ros_spinner_thread_;
  bool stop_;

  // Pointer to the model
  physics::WorldPtr world;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  // subscribe to world stats
  transport::NodePtr node;
  transport::SubscriberPtr statsSub;
  common::Time simTime;

  // Timing
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;
};

/** \} */
/// @}

}

#endif

