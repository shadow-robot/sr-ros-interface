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

/*!
  \author Stuart Glaser, modified by Ugo Cupcic

  \class sr_controller_interface::JointTrajectoryActionController

*/

#ifndef _SR_JOINT_TRAJECTORY_ACTION_CONTROLLER_HPP__
#define _SR_JOINT_TRAJECTORY_ACTION_CONTROLLER_HPP__

#include <vector>

#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <control_toolbox/limited_proxy.h>
#include <control_toolbox/pid.h>
#include <filters/filter_chain.h>
#include <pr2_controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

#include <std_msgs/Float64.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>


namespace sr_controller {

template <class Action>
class RTServerGoalHandle
{
private:
  ACTION_DEFINITION(Action);

  //typedef actionlib::ActionServer<Action>::GoalHandle GoalHandle;
  typedef actionlib::ServerGoalHandle<Action> GoalHandle;
  typedef boost::shared_ptr<Result> ResultPtr;

  uint8_t state_;

  bool req_abort_;
  bool req_succeed_;
  ResultConstPtr req_result_;

public:
  GoalHandle gh_;
  ResultPtr preallocated_result_;  // Preallocated so it can be used in realtime

  RTServerGoalHandle(GoalHandle &gh, const ResultPtr &preallocated_result = ResultPtr((Result*)NULL))
    : req_abort_(false), req_succeed_(false), gh_(gh), preallocated_result_(preallocated_result)
  {
    if (!preallocated_result_)
      preallocated_result_.reset(new Result);
  }

  void setAborted(ResultConstPtr result = ResultConstPtr((Result*)NULL))
  {
    if (!req_succeed_ && !req_abort_)
    {
      req_result_ = result;
      req_abort_ = true;
    }
  }

  void setSucceeded(ResultConstPtr result = ResultConstPtr((Result*)NULL))
  {
    if (!req_succeed_ && !req_abort_)
    {
      req_result_ = result;
      req_succeed_ = true;
    }
  }

  bool valid()
  {
    return gh_.getGoal() != NULL;
  }

  void runNonRT(const ros::TimerEvent &te)
  {
    using namespace actionlib_msgs;
    if (valid())
    {
      actionlib_msgs::GoalStatus gs = gh_.getGoalStatus();
      if (req_abort_ && gs.status == GoalStatus::ACTIVE)
      {
        if (req_result_)
          gh_.setAborted(*req_result_);
        else
          gh_.setAborted();
      }
      else if (req_succeed_ && gs.status == GoalStatus::ACTIVE)
      {
        if (req_result_)
          gh_.setSucceeded(*req_result_);
        else
          gh_.setSucceeded();
      }
    }
  }
};

class JointTrajectoryActionController : public pr2_controller_interface::Controller
{
  // Action typedefs for the original PR2 specific joint trajectory action
  typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;
  typedef RTServerGoalHandle<pr2_controllers_msgs::JointTrajectoryAction> RTGoalHandle;

  // Action typedefs for the new follow joint trajectory action
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
  typedef FJTAS::GoalHandle GoalHandleFollow;
  typedef RTServerGoalHandle<control_msgs::FollowJointTrajectoryAction> RTGoalHandleFollow;

  typedef std::map<std::string, ros::Publisher> JointPubMap;
  typedef std::map<std::string, double> JointTargetMap;

public:

  JointTrajectoryActionController();
  ~JointTrajectoryActionController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  void starting();
  void update();

private:
  int loop_count_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time last_time_;

  boost::shared_ptr<boost::thread> thread_pub_;
  JointPubMap joint_pub_;
  JointTargetMap joint_targets_;
  void publish_targets_();

  ros::NodeHandle node_;

  void commandCB(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  ros::Subscriber sub_command_;

  bool queryStateService(pr2_controllers_msgs::QueryTrajectoryState::Request &req,
                         pr2_controllers_msgs::QueryTrajectoryState::Response &resp);
  ros::ServiceServer serve_query_state_;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      pr2_controllers_msgs::JointTrajectoryControllerState> > controller_state_publisher_;

  boost::scoped_ptr<JTAS> action_server_;
  boost::scoped_ptr<FJTAS> action_server_follow_;
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void goalCBFollow(GoalHandleFollow gh);
  void cancelCBFollow(GoalHandleFollow gh);
  ros::Timer goal_handle_timer_;

  boost::shared_ptr<RTGoalHandle> rt_active_goal_;
  boost::shared_ptr<RTGoalHandleFollow> rt_active_goal_follow_;

  // ------ Mechanisms for passing the trajectory into realtime

  void commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &traj,
                         boost::shared_ptr<RTGoalHandle> gh = boost::shared_ptr<RTGoalHandle>((RTGoalHandle*)NULL),
                         boost::shared_ptr<RTGoalHandleFollow> gh_follow = boost::shared_ptr<RTGoalHandleFollow>((RTGoalHandleFollow*)NULL));

  void preemptActiveGoal();

  struct Segment
  {
    double start_time;
    double duration;
    JointTargetMap joint_targets;

    double goal_time_tolerance;

    boost::shared_ptr<RTGoalHandle> gh;
    boost::shared_ptr<RTGoalHandleFollow> gh_follow;  // Goal handle for the newer FollowJointTrajectory action
  };
  typedef std::vector<Segment> SpecifiedTrajectory;

  realtime_tools::RealtimeBox<
    boost::shared_ptr<const SpecifiedTrajectory> > current_trajectory_box_;
};

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
