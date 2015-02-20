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

#include <sr_gazebo_plugins/gazebo_ros_controller_manager.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <set>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/Base.hh>

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <angles/angles.h>
#include <urdf/model.h>

#include <sr_hardware_interface/sr_actuator.hpp>

using namespace ros_ethercat_model;
using namespace std;
using boost::ptr_unordered_map;

namespace gazebo
{

GazeboRosControllerManager::GazeboRosControllerManager()
  : state_(NULL), cm_(NULL), fake_state_(NULL), rosnode_(NULL), stop_(false)
{
}

GazeboRosControllerManager::~GazeboRosControllerManager()
{
  ROS_DEBUG("Calling FiniChild in GazeboRosControllerManager");

#ifdef USE_CBQ
  controller_manager_queue_.clear();
  controller_manager_queue_.disable();
  controller_manager_callback_queue_thread_.join();
#endif
  stop_ = true;
  ros_spinner_thread_.join();
  ROS_DEBUG("spinner terminated");
  self_test_.reset();
  delete cm_;
  delete rosnode_;
  delete state_;
}

void GazeboRosControllerManager::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent model
  string modelName = _sdf->GetParent()->Get<string>("name");

  // Get the world name.
  world = _parent->GetWorld();

  // Get a pointer to the model
  parent_model_ = _parent;

  // Error message if the model couldn't be found
  if (!parent_model_)
    ROS_ERROR("Unable to get parent model");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosControllerManager::UpdateChild, this));
  ROS_DEBUG_STREAM("plugin model name: " << modelName);

  if (getenv("CHECK_SPEEDUP"))
  {
    wall_start_ = world->GetRealTime().Double();
    sim_start_ = world->GetSimTime().Double();
  }

  // get parameter name
  robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
    robotNamespace = _sdf->GetElement("robotNamespace")->Get<string>();

  robotParam = "robot_description";
  if (_sdf->HasElement("robotParam"))
    robotParam = _sdf->GetElement("robotParam")->Get<string>();

  robotParam = robotNamespace + "/" + robotParam;

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }
  rosnode_ = new ros::NodeHandle(robotNamespace);
  ROS_INFO("starting gazebo_ros_controller_manager plugin in ns: %s", robotNamespace.c_str());

  // Use the robots namespace, not gazebos
  self_test_.reset(new shadow_robot::SrSelfTest(true, robotNamespace));

  // ros_ethercat calls ros::spin(), we'll thread out one spinner here to mimic that
  ros_spinner_thread_ = boost::thread(boost::bind(&GazeboRosControllerManager::ControllerManagerROSThread, this));

  rosnode_->param("gazebo/start_robot_calibrated", fake_calibration_, true);

  // setup the robot state
  ReadPr2Xml();

  // The gazebo joints and mechanism joints should match up.
  if (state_ != NULL) // could be NULL if ReadPr2Xml is unsuccessful
  {
    for (ptr_unordered_map<string, JointState>::iterator it = state_->model_.joint_states_.begin();
         it != state_->model_.joint_states_.end();
         ++it)
    {
      // fill the gazebo joints pointer vector
      physics::JointPtr joint = parent_model_->GetJoint(it->first);
      if (joint)
        joints_.push_back(joint);
      else
        ROS_ERROR_STREAM("A joint named " << it->first << " is not part of Mechanism Controlled joints");
    }
  }

  // Get the Gazebo simulation period
  ros::Duration gazebo_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());

  // Decide the plugin control period
  if (_sdf->HasElement("controlPeriod"))
  {
    control_period_ = ros::Duration(_sdf->Get<double>("controlPeriod"));

    // Check the period against the simulation period
    if (control_period_ < gazebo_period)
    {
      ROS_DEBUG_STREAM("Desired controller update period (" << control_period_
                       << " s) is faster than the gazebo simulation period (" << gazebo_period << " s).");
    }
    else if (control_period_ > gazebo_period)
    {
      ROS_DEBUG_STREAM("Desired controller update period (" << control_period_
                       << " s) is slower than the gazebo simulation period (" << gazebo_period << " s).");
    }
  }
  else
  {
    control_period_ = gazebo_period;
    ROS_DEBUG_STREAM("Control period not found in URDF/SDF, defaulting to Gazebo period of "
                     << control_period_);
  }

#ifdef USE_CBQ
  // start custom queue for controller manager
  controller_manager_callback_queue_thread_ =
    boost::thread(boost::bind(&GazeboRosControllerManager::ControllerManagerQueueThread, this));
#endif

}

// Called on world reset

void GazeboRosControllerManager::ResetChild()
{
  // Reset timing variables to not pass negative update periods to controllers on world reset
  last_update_sim_time_ros_ = ros::Time();
  last_write_sim_time_ros_ = ros::Time();
}

void GazeboRosControllerManager::UpdateChild()
{
  if (world->IsPaused() || !fake_state_ || !cm_)
    return;

  // Get the simulation time and period
  common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;
  if (getenv("CHECK_SPEEDUP"))
  {
    double wall_elapsed = world->GetRealTime().Double() - wall_start_;
    double sim_elapsed = world->GetSimTime().Double() - sim_start_;
    ROS_DEBUG_STREAM(" real time: " << wall_elapsed
                     << "  sim time: " << sim_elapsed
                     << "  speed up: " << sim_elapsed / wall_elapsed);
  }
  ROS_ASSERT(joints_.size() == fake_state_->joint_states_.size());

  // Check if we should update the controllers
  if (sim_period >= control_period_)
  {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    //--------------------------------------------------
    //  Pushes out simulation state
    //--------------------------------------------------

    // Copies the state from the gazebo joints into the mechanism joints.
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      JointState *fst = fake_state_->getJointState(joints_[i]->GetName());

      if (joints_[i]->HasType(physics::Base::HINGE_JOINT))
        fst->position_ += angles::shortest_angular_distance(fst->position_, joints_[i]->GetAngle(0).Radian());
      else if (joints_[i]->HasType(physics::Base::SLIDER_JOINT))
        fst->position_ = joints_[i]->GetAngle(0).Radian();

      fst->velocity_ = joints_[i]->GetVelocity(0);
    }

    //--------------------------------------------------
    //  Runs Mechanism Control
    //--------------------------------------------------
    fake_state_->current_time_ = ros::Time(world->GetSimTime().Double());
    cm_->update(sim_time_ros, sim_period);
  }

  //--------------------------------------------------
  //  Takes in actuation commands
  //--------------------------------------------------

  // Copies the commands from the mechanism joints into the gazebo joints.
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    string joint_name = joints_[i]->GetName();

    JointState *fst = fake_state_->getJointState(joint_name);

    // joints 1 and 2 of fingers ff, mf, rf, lf share an actuator
    // at this moment only commanded_effort_ in joint 2 is up to date
    double effort = fst->commanded_effort_;

    if(joint_name.size() > 3)
    {
      if (joint_name[joint_name.size() - 1] == '2' && (joint_name[joint_name.size() -4] == 'F' ||
						       joint_name[joint_name.size() -4] == 'M' ||
						       joint_name[joint_name.size() -4] == 'R' ||
						       joint_name[joint_name.size() -4] == 'L'))
      {
	joint_name[joint_name.size() - 1] = '1';

	JointState *fst1 = fake_state_->getJointState(joint_name);
	effort = fst1->commanded_effort_;
      }
    }

    double damping_coef = 0;
    if (fst->joint_->dynamics)
      damping_coef = fst->joint_->dynamics->damping;
    
    double current_velocity = joints_[i]->GetVelocity(0);
    // using implicit spring damper from gazebo instead of explicit.
    //double damping_force = damping_coef * current_velocity;
    //double effort_command = effort - damping_force;
    double effort_command = effort;
    //enforce limits (are these limites enforced in gazebo ?)
    //if (fst->joint_->limits)
    //{
      //double effort_limit= fst->joint_->limits->effort;
      //effort_command=std::min(std::max(effort_command, -effort_limit), effort_limit);
    //}
    joints_[i]->SetForce(0, effort_command);
  }
  last_write_sim_time_ros_ = sim_time_ros;
}

void GazeboRosControllerManager::ReadPr2Xml()
{

  string urdf_param_name;
  string urdf_string;
  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    ROS_DEBUG_STREAM("gazebo controller manager plugin is waiting for urdf: " << robotParam << " on the param server");
    if (rosnode_->searchParam(robotParam, urdf_param_name))
    {
      rosnode_->getParam(urdf_param_name, urdf_string);
      ROS_DEBUG_STREAM("found upstream");
    }
    else
    {
      rosnode_->getParam(robotParam, urdf_string);
      ROS_DEBUG_STREAM("found in node namespace");
    }
    ROS_DEBUG_STREAM(robotParam << "\n------\n" << urdf_param_name << "\n------\n" << urdf_string);
    usleep(100000);
  }
  ROS_DEBUG_STREAM("gazebo controller manager got pr2.xml from param server, parsing it...");

  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf_string.c_str()) && doc.Error())
  {
    ROS_ERROR_STREAM("Could not load the gazebo controller manager plugin's configuration file: " << urdf_string);
  }
  else
  {
    // Pulls out the list of actuators used in the robot configuration.

    struct GetActuators : public TiXmlVisitor
    {
      set<string> actuators;

      virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
      {
        if (elt.ValueStr() == string("actuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() == string("rightActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() == string("leftActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        return true;
      }
    } get_actuators;
    doc.RootElement()->Accept(&get_actuators);

    state_ = new RosEthercat(*rosnode_, "", true, doc.RootElement());
    fake_state_ = &state_->model_;

    // load a controller manager
    cm_ = new controller_manager::ControllerManager(state_, *rosnode_);

    fake_state_->current_time_ = ros::Time(world->GetSimTime().Double());

    for (ptr_unordered_map<string, JointState>::iterator jit = state_->model_.joint_states_.begin();
         jit != state_->model_.joint_states_.end();
         ++jit)
    {
      jit->second->calibrated_ = fake_calibration_;
    }
  }

}

#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// custom callback queue

void GazeboRosControllerManager::ControllerManagerQueueThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    controller_manager_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

void GazeboRosControllerManager::ControllerManagerROSThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  while (rosnode_->ok() && !stop_)
  {
    self_test_->checkTest();
    usleep(1000);
    ros::spinOnce();
  }
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosControllerManager)
} // namespace gazebo
