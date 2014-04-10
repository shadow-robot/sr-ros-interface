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

//#include <gazebo/XMLConfig.hh>
//#include "physics/physics.h"
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
#include <map>

#include <sr_hardware_interface/sr_actuator.hpp>

using namespace ros_ethercat_mechanism_model;
namespace gazebo {

GazeboRosControllerManager::GazeboRosControllerManager()
{
}


/// \brief callback for setting models joints states
//bool setModelsJointsStates(pr2_gazebo_plugins::SetModelsJointsStates::Request &req,
//                           pr2_gazebo_plugins::SetModelsJointsStates::Response &res)
//{
//
//  return true;
//}


GazeboRosControllerManager::~GazeboRosControllerManager()
{
  ROS_DEBUG("Calling FiniChild in GazeboRosControllerManager");

  //pr2_hardware_interface::ActuatorMap::const_iterator it;
  //for (it = hw_.actuators_.begin(); it != hw_.actuators_.end(); ++it)
  //  delete it->second; // why is this causing double free corruption?
  delete state_;
#ifdef USE_CBQ
  this->controller_manager_queue_.clear();
  this->controller_manager_queue_.disable();
  this->controller_manager_callback_queue_thread_.join();
#endif
  this->ros_spinner_thread_.join();




  delete this->cm_;
  delete this->rosnode_;
  delete this->fake_state_;
}


void GazeboRosControllerManager::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Get the world name.
  this->world = _parent->GetWorld();

  // Get a pointer to the model
  this->parent_model_ = _parent;

  // Error message if the model couldn't be found
  if (!this->parent_model_)
    gzerr << "Unable to get parent model\n";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosControllerManager::UpdateChild, this));
  gzdbg << "plugin model name: " << modelName << "\n";




  if (getenv("CHECK_SPEEDUP"))
  {
    wall_start_ = this->world->GetRealTime().Double();
    sim_start_  = this->world->GetSimTime().Double();
  }

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  //if (this->updatePeriod > 0 &&
  //    (this->world->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
  //  ROS_ERROR("gazebo_ros_force controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");





  // get parameter name
  this->robotNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();

  this->robotParam = "robot_description";
  if (_sdf->HasElement("robotParam"))
    this->robotParam = _sdf->GetElement("robotParam")->Get<std::string>();

  this->robotParam = this->robotNamespace+"/" + this->robotParam;

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);
  ROS_INFO("starting gazebo_ros_controller_manager plugin in ns: %s",this->robotNamespace.c_str());

  // Use the robots namespace, not gazebos
  self_test_.reset(new shadow_robot::SrSelfTest(true, this->robotNamespace));

  // ros_ethercat calls ros::spin(), we'll thread out one spinner here to mimic that
  this->ros_spinner_thread_ = boost::thread( boost::bind( &GazeboRosControllerManager::ControllerManagerROSThread,this ) );

  this->rosnode_->param("gazebo/start_robot_calibrated",this->fake_calibration_,true);

  // read pr2 urdf
  // setup actuators, then setup mechanism control node
  ReadPr2Xml();

  // The gazebo joints and mechanism joints should match up.
  if (this->cm_->state_ != NULL) // could be NULL if ReadPr2Xml is unsuccessful
  {
    for (boost::unordered_map<std::string, JointState>::iterator it = state_->model_.joint_states_.begin(); it != state_->model_.joint_states_.end(); ++it)
    {


      // fill in gazebo joints pointer
      gazebo::physics::JointPtr joint = this->parent_model_->GetJoint(it->first);
      if (joint)
      {
        this->joints_.push_back(joint);
      }
      else
      {
        //ROS_WARN("A joint named \"%s\" is not part of Mechanism Controlled joints.\n", joint_name.c_str());
        //this->joints_.push_back(NULL);  // FIXME: cannot be null, must be an empty boost shared pointer
        this->joints_.push_back(gazebo::physics::JointPtr());  // FIXME: cannot be null, must be an empty boost shared pointer
        ROS_ERROR("A joint named \"%s\" is not part of Mechanism Controlled joints.\n", it->first.c_str());
      }
    }
  }

#ifdef USE_CBQ
  // start custom queue for controller manager
  this->controller_manager_callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosControllerManager::ControllerManagerQueueThread,this ) );
#endif

}


void GazeboRosControllerManager::UpdateChild()
{
  if (this->world->IsPaused()) return;

  if (getenv("CHECK_SPEEDUP"))
  {
    double wall_elapsed = this->world->GetRealTime().Double() - wall_start_;
    double sim_elapsed  = this->world->GetSimTime().Double()  - sim_start_;
    std::cout << " real time: " <<  wall_elapsed
              << "  sim time: " <<  sim_elapsed
              << "  speed up: " <<  sim_elapsed / wall_elapsed
              << std::endl;
  }
  assert(this->joints_.size() == this->fake_state_->joint_states_.size());

  //--------------------------------------------------
  //  Pushes out simulation state
  //--------------------------------------------------

  //ROS_ERROR("joints_.size()[%d]",(int)this->joints_.size());
  // Copies the state from the gazebo joints into the mechanism joints.
  vector<gazebo::physics::JointPtr>::iterator git = joints_.begin();
  boost::unordered_map<std::string, JointState>::iterator fit = fake_state_->joint_states_.begin();
  while (git != joints_.end() &&
         fit != fake_state_->joint_states_.end())
  {
    if (!(*git))
      continue;

    fit->second.measured_effort_ = fit->second.commanded_effort_;

    if ((*git)->HasType(gazebo::physics::Base::HINGE_JOINT))
    {
      fit->second.position_ = fit->second.position_ +
                    angles::shortest_angular_distance(fit->second.position_, (*git)->GetAngle(0).Radian());
      fit->second.velocity_ = (*git)->GetVelocity(0);
    }
    else if ((*git)->HasType(gazebo::physics::Base::SLIDER_JOINT))
    {
        fit->second.position_ = (*git)->GetAngle(0).Radian();
        fit->second.velocity_ = (*git)->GetVelocity(0);
    }
  }

  // Reverses the transmissions to propagate the joint position into the actuators.
  this->fake_state_->propagateJointPositionToActuatorPosition();

  //--------------------------------------------------
  //  Runs Mechanism Control
  //--------------------------------------------------
  this->fake_state_->current_time_ = ros::Time(this->world->GetSimTime().Double());
  try
  {
    if (this->cm_->state_ != NULL) // could be NULL if ReadPr2Xml is unsuccessful
      this->cm_->update(fake_state_->current_time_, ros::Duration(1e+6));
  }
  catch (const char* c)
  {
    if (strcmp(c,"dividebyzero")==0)
      ROS_WARN("pid controller reports divide by zero error");
    else
      ROS_WARN("unknown const char* exception: %s", c);
  }

  //--------------------------------------------------
  //  Takes in actuation commands
  //--------------------------------------------------

  // Reverses the transmissions to propagate the actuator commands into the joints.
  this->fake_state_->propagateActuatorEffortToJointEffort();

  // Copies the commands from the mechanism joints into the gazebo joints.
  git = joints_.begin();
  fit = fake_state_->joint_states_.begin();
  boost::unordered_map<std::string, JointState>::iterator rit = state_->model_.joint_states_.begin();

  while (git != joints_.end() &&
         fit != fake_state_->joint_states_.end() &&
         rit != state_->model_.joint_states_.end())
  {
    if (!*git)
      continue;

    double effort = fit->second.commanded_effort_;

    double damping_coef = 0;
    if (rit->second.joint_->dynamics)
      damping_coef = rit->second.joint_->dynamics->damping;

    if ((*git)->HasType(gazebo::physics::Base::HINGE_JOINT) || (*git)->HasType(gazebo::physics::Base::SLIDER_JOINT))
    {
      double current_velocity = (*git)->GetVelocity(0);
      double damping_force = damping_coef * current_velocity;
      double effort_command = effort - damping_force;
      (*git)->SetForce(0,effort_command);
    }
    ++git;
    ++fit;
    ++rit;
  }
}

void GazeboRosControllerManager::ReadPr2Xml()
{

  std::string urdf_param_name;
  std::string urdf_string;
  // search and wait for robot_description on param server
  while(urdf_string.empty())
  {
    ROS_DEBUG("gazebo controller manager plugin is waiting for urdf: %s on the param server.", this->robotParam.c_str());
    if (this->rosnode_->searchParam(this->robotParam,urdf_param_name))
    {
      this->rosnode_->getParam(urdf_param_name,urdf_string);
      ROS_DEBUG("found upstream\n%s\n------\n%s\n------\n%s",this->robotParam.c_str(),urdf_param_name.c_str(),urdf_string.c_str());
    }
    else
    {
      this->rosnode_->getParam(this->robotParam,urdf_string);
      ROS_DEBUG("found in node namespace\n%s\n------\n%s\n------\n%s",this->robotParam.c_str(),urdf_param_name.c_str(),urdf_string.c_str());
    }
    usleep(100000);
  }
  ROS_DEBUG("gazebo controller manager got pr2.xml from param server, parsing it...");

  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf_string.c_str()) && doc.Error())
  {
    ROS_ERROR("Could not load the gazebo controller manager plugin's configuration file: %s\n",
            urdf_string.c_str());
  }
  else
  {
    //doc.Print();
    //std::cout << *(doc.RootElement()) << std::endl;

    // Pulls out the list of actuators used in the robot configuration.
    struct GetActuators : public TiXmlVisitor
    {
      std::set<std::string> actuators;
      virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
      {
        if (elt.ValueStr() == std::string("actuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() == std::string("rightActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        else if (elt.ValueStr() == std::string("leftActuator") && elt.Attribute("name"))
          actuators.insert(elt.Attribute("name"));
        return true;
      }
    } get_actuators;
    doc.RootElement()->Accept(&get_actuators);

    state_ = new ros_ethercat(*rosnode_, "", true, doc.RootElement());
    fake_state_ = new Robot(doc.RootElement());

    // load a controller manager
    cm_ = new controller_manager::ControllerManager(state_, *rosnode_);

    fake_state_->current_time_ = ros::Time(world->GetSimTime().Double());
    if (fake_state_->current_time_ < ros::Time(0.001))
      fake_state_->current_time_ = ros::Time(0.001); // hardcoded to minimum of 1ms on start up

    // Places the found actuators into the hardware interface.
    std::set<std::string>::iterator it;
    for (it = get_actuators.actuators.begin(); it != get_actuators.actuators.end(); ++it)
    {
      fake_state_->actuators_[*it];
    }

    boost::unordered_map<std::string, JointState>::iterator jit = state_->model_.joint_states_.begin();
    while (jit != state_->model_.joint_states_.end())
    {
      jit->second.calibrated_ = fake_calibration_;
      ++jit;
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

  while (this->rosnode_->ok())
  {
    this->controller_manager_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

void GazeboRosControllerManager::ControllerManagerROSThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  //ros::Rate rate(1000);

  while (this->rosnode_->ok())
  {
    self_test_->checkTest();
    //rate.sleep(); // using rosrate gets stuck on model delete
    usleep(1000);
    ros::spinOnce();
  }
}
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosControllerManager)
} // namespace gazebo
