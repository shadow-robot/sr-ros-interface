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

/*
 * Author: Stuart Glaser, modified by Ugo Cupcic
 */

#include "sr_mechanism_controllers/joint_trajectory_action_controller.hpp"
#include <sstream>
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

//PLUGINLIB_DECLARE_CLASS(sr_mechanism_controllers, JointTrajectoryActionController, controller::JointTrajectoryActionController, pr2_controller_interface::Controller)
PLUGINLIB_EXPORT_CLASS( sr_controller::JointTrajectoryActionController, pr2_controller_interface::Controller)

namespace sr_controller {

  JointTrajectoryActionController::JointTrajectoryActionController()
    : loop_count_(0), robot_(NULL)
  {
  }

  JointTrajectoryActionController::~JointTrajectoryActionController()
  {
    sub_command_.shutdown();
    serve_query_state_.shutdown();
    action_server_.reset();
    action_server_follow_.reset();
  }

  bool JointTrajectoryActionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    node_ = n;
    robot_ = robot;

    //Create a map of joint names to their command publishers
    //Hand joints
    //TODO: this could be read from the controller manager
    // rosservice call /pr2_controller_manager/list_controllers
    std::string hand_names[] = {
      "ffj0", "ffj3", "ffj4",
      "lfj0", "lfj3", "lfj4", "lfj5",
      "mfj0", "mfj3", "mfj4",
      "rfj0", "rfj3", "rfj4",
      "thj1", "thj2", "thj3", "thj4", "thj5",
      "wrj1", "wrj2"
    };
    for (size_t i = 0; i < 20; i++)
    {
      joint_pub_[hand_names[i]] = node_.advertise<std_msgs::Float64>(
        "/sh_"+hand_names[i]+"_mixed_position_velocity_controller/command", 2);

      joint_pub_[ boost::to_upper_copy(hand_names[i]) ] = node_.advertise<std_msgs::Float64>(
        "/sh_"+hand_names[i]+"_mixed_position_velocity_controller/command", 2);
    }

    //Arm joints
    std::string arm_names[] = {"sr", "ss", "es", "er"};
    for (size_t i = 0; i < 4; i++)
    {
      joint_pub_[arm_names[i]] = node_.advertise<std_msgs::Float64>(
        "/sa_"+arm_names[i]+"_position_controller/command", 2);
    }

    //Arm joints: 2 naming conventions
    std::string arm_names_2[] = {"ShoulderJRotate", "ShoulderJSwing", "ElbowJSwing", "ElbowJRotate"};
    for (size_t i = 0; i < 4; i++)
    {
      joint_pub_[arm_names_2[i]] = node_.advertise<std_msgs::Float64>(
        "/sa_"+arm_names[i]+"_position_controller/command", 2);
    }

    //create a dummy trajectory
    boost::shared_ptr<SpecifiedTrajectory> traj_ptr(new SpecifiedTrajectory(1));
    SpecifiedTrajectory &traj = *traj_ptr;
    traj[0].start_time = robot_->getTime().toSec();
    traj[0].duration = 0.0;
    current_trajectory_box_.set(traj_ptr);

    sub_command_ = node_.subscribe("command", 1, &JointTrajectoryActionController::commandCB, this);
    serve_query_state_ = node_.advertiseService(
      "query_state", &JointTrajectoryActionController::queryStateService, this);

    controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointTrajectoryControllerState>
      (node_, "state", 1));
    controller_state_publisher_->lock();
    //@todo change controller state msg?
    controller_state_publisher_->unlock();

    action_server_.reset(new JTAS(node_, "joint_trajectory_action",
                                  boost::bind(&JointTrajectoryActionController::goalCB, this, _1),
                                  boost::bind(&JointTrajectoryActionController::cancelCB, this, _1),
                                  false));
    action_server_follow_.reset(new FJTAS(node_, "follow_joint_trajectory",
                                          boost::bind(&JointTrajectoryActionController::goalCBFollow, this, _1),
                                          boost::bind(&JointTrajectoryActionController::cancelCBFollow, this, _1),
                                          false));
    action_server_->start();
    action_server_follow_->start();

    return true;
  }

  void JointTrajectoryActionController::starting()
  {
    last_time_ = robot_->getTime();

    thread_pub_.reset(new boost::thread(boost::bind(&JointTrajectoryActionController::publish_targets_, this)));
  }

  void JointTrajectoryActionController::update()
  {
    ros::Time time = robot_->getTime();
    ros::Duration dt = time - last_time_;
    last_time_ = time;

    boost::shared_ptr<RTGoalHandle> current_active_goal(rt_active_goal_);
    boost::shared_ptr<RTGoalHandleFollow> current_active_goal_follow(rt_active_goal_follow_);

    boost::shared_ptr<const SpecifiedTrajectory> traj_ptr;
    current_trajectory_box_.get(traj_ptr);
    if (!traj_ptr)
      ROS_FATAL("The current trajectory can never be null");

    // Only because this is what the code originally looked like.
    const SpecifiedTrajectory &traj = *traj_ptr;

    // ------ Finds the current segment

    // Determines which segment of the trajectory to use.  (Not particularly realtime friendly).
    int seg = -1;
    while (seg + 1 < (int)traj.size() &&
           traj[seg+1].start_time < time.toSec())
    {
      ++seg;
    }

    if (seg == -1)
    {
      if (traj.size() == 0)
        ROS_ERROR("No segments in the trajectory");
      else
        ROS_ERROR("No earlier segments.  First segment starts at %.3lf (now = %.3lf)", traj[0].start_time, time.toSec());
      return;
    }

    //Update the joint_targets_ with the one from the current segment
    JointTargetMap::iterator it;
    Segment tmp_seg = traj[seg];
    for( it = tmp_seg.joint_targets.begin(); it != tmp_seg.joint_targets.end(); ++it )
    {
      joint_targets_[it->first] = it->second;
    }

    // ------ Determines if the goal has failed or succeeded
    if ((traj[seg].gh && traj[seg].gh == current_active_goal) ||
        (traj[seg].gh_follow && traj[seg].gh_follow == current_active_goal_follow))
    {
      ros::Time end_time(traj[seg].start_time + traj[seg].duration);
      if (time <= end_time)
      {
        //@todo check traj constraints
      }
      else if (seg == (int)traj.size() - 1)
      {
        //@todo Checks that we have ended inside the goal tolerances
        bool inside_goal_constraints = true;

        if (inside_goal_constraints)
        {
          rt_active_goal_.reset();
          rt_active_goal_follow_.reset();
          if (traj[seg].gh)
            traj[seg].gh->setSucceeded();
          else if (traj[seg].gh_follow) {
            traj[seg].gh_follow->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
            traj[seg].gh_follow->setSucceeded(traj[seg].gh_follow->preallocated_result_);
          }
        }
        else if (time < end_time + ros::Duration(traj[seg].goal_time_tolerance))
        {
          // Still have some time left to make it.
        }
        else
        {
          //ROS_WARN("Aborting because we wound up outside the goal constraints");
          rt_active_goal_.reset();
          rt_active_goal_follow_.reset();
          if (traj[seg].gh)
            traj[seg].gh->setAborted();
          else if (traj[seg].gh_follow) {
            traj[seg].gh_follow->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
            traj[seg].gh_follow->setAborted(traj[seg].gh_follow->preallocated_result_);
          }
        }
      }
    }

    // ------ State publishing

    if (loop_count_ % 10 == 0)
    {
      if (controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;

        //@todo publish meaningful state data

        controller_state_publisher_->unlockAndPublish();
      }
    }

    ++loop_count_;
  }

  void JointTrajectoryActionController::commandCB(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
  {
    preemptActiveGoal();
    commandTrajectory(msg);
  }

  void JointTrajectoryActionController::commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg,
                                                          boost::shared_ptr<RTGoalHandle> gh,
                                                          boost::shared_ptr<RTGoalHandleFollow> gh_follow)
  {
    assert(!gh || !gh_follow);
    ros::Time time = last_time_ + ros::Duration(0.01);
    ROS_DEBUG("Figuring out new trajectory at %.3lf, with data from %.3lf",
              time.toSec(), msg->header.stamp.toSec());

    boost::shared_ptr<SpecifiedTrajectory> new_traj_ptr(new SpecifiedTrajectory);
    SpecifiedTrajectory &new_traj = *new_traj_ptr;

    // ------ If requested, performs a stop

    if (msg->points.empty())
    {
      starting();
      return;
    }

    if (gh_follow)
    {
      JointTargetMap::iterator it;
      for (size_t k = 0; k < msg->joint_names.size(); ++k)
      {
        if( joint_pub_.find(msg->joint_names[k]) == joint_pub_.end() )
        {
          ROS_ERROR("Unable to locate joint %s in the commanded trajectory.", msg->joint_names[k].c_str());
          if (gh)
            gh->setAborted();
          else if (gh_follow)
          {
            gh_follow->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
            gh_follow->setAborted(gh_follow->preallocated_result_);
          }
          return;

        }
      }
    }

    // ------ Grabs the trajectory that we're currently following.

    boost::shared_ptr<const SpecifiedTrajectory> prev_traj_ptr;
    current_trajectory_box_.get(prev_traj_ptr);
    if (!prev_traj_ptr)
    {
      ROS_FATAL("The current trajectory can never be null");
      return;
    }
    const SpecifiedTrajectory &prev_traj = *prev_traj_ptr;

    // ------ Copies over the segments from the previous trajectory that are still useful.

    // Useful segments are still relevant after the current time.
    int first_useful = -1;
    while (first_useful + 1 < (int)prev_traj.size() &&
           prev_traj[first_useful + 1].start_time <= time.toSec())
    {
      ++first_useful;
    }

    // Useful segments are not going to be completely overwritten
    int last_useful = -1;
    double msg_start_time;
    if (msg->header.stamp == ros::Time(0.0))
      msg_start_time = time.toSec();
    else
      msg_start_time = msg->header.stamp.toSec();
    /*
      if (msg->points.size() > 0)
      msg_start_time += msg->points[0].time_from_start.toSec();
    */

    while (last_useful + 1 < (int)prev_traj.size() &&
           prev_traj[last_useful + 1].start_time < msg_start_time)
    {
      ++last_useful;
    }

    if (last_useful < first_useful)
      first_useful = last_useful;

    // Copies over the old segments that were determined to be useful.
    for (int i = std::max(first_useful,0); i <= last_useful; ++i)
    {
      new_traj.push_back(prev_traj[i]);
    }

    // We always save the last segment so that we know where to stop if
    // there are no new segments.
    if (new_traj.size() == 0)
      new_traj.push_back(prev_traj[prev_traj.size() - 1]);

    // ------ Determines when and where the new segments start

    // Finds the end conditions of the final segment
    Segment &last = new_traj[new_traj.size() - 1];
    double t = (msg->header.stamp == ros::Time(0.0) ? time.toSec() : msg->header.stamp.toSec())
      - last.start_time;
    ROS_DEBUG("Initial conditions at %.3f for new set of splines:", t);

    // ------ Tacks on the new segments
    std::vector<double> durations(msg->points.size());
    if (msg->points.size() > 0)
      durations[0] = msg->points[0].time_from_start.toSec();
    for (size_t i = 1; i < msg->points.size(); ++i)
      durations[i] = (msg->points[i].time_from_start - msg->points[i-1].time_from_start).toSec();

    for (size_t i = 0; i < msg->points.size(); ++i)
    {
      Segment seg;

      if (msg->header.stamp == ros::Time(0.0))
        seg.start_time = (time + msg->points[i].time_from_start).toSec() - durations[i];
      else
        seg.start_time = (msg->header.stamp + msg->points[i].time_from_start).toSec() - durations[i];
      seg.duration = durations[i];
      seg.gh = gh;
      seg.gh_follow = gh_follow;

      for(size_t joint_index = 0; joint_index < msg->joint_names.size(); ++joint_index)
      {
        seg.joint_targets[ msg->joint_names[joint_index] ] = msg->points[i].positions[joint_index];
      }

      new_traj.push_back( seg );
    }

    //ROS_ERROR("Last segment goal id: %s", new_traj[new_traj.size()-1].gh->gh_.getGoalID().id.c_str());

    // ------ Commits the new trajectory

    if (!new_traj_ptr)
    {
      ROS_ERROR("The new trajectory was null!");
      return;
    }

    current_trajectory_box_.set(new_traj_ptr);
  }

  bool JointTrajectoryActionController::queryStateService(
    pr2_controllers_msgs::QueryTrajectoryState::Request &req,
    pr2_controllers_msgs::QueryTrajectoryState::Response &resp)
  {
    boost::shared_ptr<const SpecifiedTrajectory> traj_ptr;
    current_trajectory_box_.get(traj_ptr);
    if (!traj_ptr)
    {
      ROS_FATAL("The current trajectory can never be null");
      return false;
    }
    const SpecifiedTrajectory &traj = *traj_ptr;

    // Determines which segment of the trajectory to use
    int seg = -1;
    while (seg + 1 < (int)traj.size() &&
           traj[seg+1].start_time < req.time.toSec())
    {
      ++seg;
    }
    if (seg == -1)
      return false;

    //@todo mutex this
    JointTargetMap::iterator it;
    for( it = joint_targets_.begin(); it != joint_targets_.end(); ++it )
    {
      resp.name.push_back(it->first);
      resp.position.push_back(it->second);
    }

    return true;
  }

  void JointTrajectoryActionController::preemptActiveGoal()
  {
    boost::shared_ptr<RTGoalHandle> current_active_goal(rt_active_goal_);
    boost::shared_ptr<RTGoalHandleFollow> current_active_goal_follow(rt_active_goal_follow_);

    // Cancels the currently active goal.
    if (current_active_goal)
    {
      // Marks the current goal as canceled.
      rt_active_goal_.reset();
      current_active_goal->gh_.setCanceled();
    }
    if (current_active_goal_follow)
    {
      rt_active_goal_follow_.reset();
      current_active_goal_follow->gh_.setCanceled();
    }
  }


  template <class Enclosure, class Member>
  static boost::shared_ptr<Member> share_member(boost::shared_ptr<Enclosure> enclosure, Member &member)
  {
    actionlib::EnclosureDeleter<Enclosure> d(enclosure);
    boost::shared_ptr<Member> p(&member, d);
    return p;
  }

  void JointTrajectoryActionController::goalCB(GoalHandle gh)
  {
    //@todo Ensures that the joints in the goal match the joints we are commanding.
    preemptActiveGoal();

    gh.setAccepted();
    boost::shared_ptr<RTGoalHandle> rt_gh(new RTGoalHandle(gh));

    // Sends the trajectory along to the controller
    goal_handle_timer_ = node_.createTimer(ros::Duration(0.01), &RTGoalHandle::runNonRT, rt_gh);
    commandTrajectory(share_member(gh.getGoal(), gh.getGoal()->trajectory), rt_gh);
    rt_active_goal_ = rt_gh;
    goal_handle_timer_.start();
  }

  void JointTrajectoryActionController::goalCBFollow(GoalHandleFollow gh)
  {
    //@todo Ensures that the joints in the goal match the joints we are commanding.
    preemptActiveGoal();

    gh.setAccepted();
    boost::shared_ptr<RTGoalHandleFollow> rt_gh(new RTGoalHandleFollow(gh));

    // Sends the trajectory along to the controller
    goal_handle_timer_ = node_.createTimer(ros::Duration(0.01), &RTGoalHandleFollow::runNonRT, rt_gh);
    commandTrajectory(share_member(gh.getGoal(), gh.getGoal()->trajectory),
                      boost::shared_ptr<RTGoalHandle>((RTGoalHandle*)NULL),
                      rt_gh);
    rt_active_goal_follow_ = rt_gh;
    goal_handle_timer_.start();
  }

  void JointTrajectoryActionController::cancelCB(GoalHandle gh)
  {
    boost::shared_ptr<RTGoalHandle> current_active_goal(rt_active_goal_);
    if (current_active_goal && current_active_goal->gh_ == gh)
    {
      rt_active_goal_.reset();

      trajectory_msgs::JointTrajectory::Ptr empty(new trajectory_msgs::JointTrajectory);
      JointTargetMap::iterator it;
      for( it = joint_targets_.begin(); it != joint_targets_.end(); ++it )
      {
        empty->joint_names.push_back(it->first);
      }
      commandTrajectory(empty);

      // Marks the current goal as canceled.
      current_active_goal->gh_.setCanceled();
    }
  }

  void JointTrajectoryActionController::cancelCBFollow(GoalHandleFollow gh)
  {
    boost::shared_ptr<RTGoalHandleFollow> current_active_goal(rt_active_goal_follow_);
    if (current_active_goal && current_active_goal->gh_ == gh)
    {
      rt_active_goal_follow_.reset();

      trajectory_msgs::JointTrajectory::Ptr empty(new trajectory_msgs::JointTrajectory);
      JointTargetMap::iterator it;
      for( it = joint_targets_.begin(); it != joint_targets_.end(); ++it )
      {
        empty->joint_names.push_back(it->first);
      }
      commandTrajectory(empty);

      // Marks the current goal as canceled.
      current_active_goal->gh_.setCanceled();
    }
  }

  void JointTrajectoryActionController::publish_targets_()
  {
    ros::Rate rate(20.0); //@todo read rate from param
    std_msgs::Float64 msg;

    while( ros::ok() )
    {
      boost::shared_ptr<RTGoalHandleFollow> current_active_goal(rt_active_goal_follow_);
      if( current_active_goal == NULL )
      {
        rate.sleep();
        continue;
      }
      if( current_active_goal->gh_.getGoalStatus().status !=  actionlib_msgs::GoalStatus::ACTIVE)
      {
        rate.sleep();
        continue;
      }

      std::stringstream dbg;

      //@todo mutex this
      JointTargetMap::iterator it;
      for( it = joint_targets_.begin(); it != joint_targets_.end(); ++it )
      {
        msg.data = it->second;
        joint_pub_[it->first].publish(msg);

        dbg << " [" << it->first << "]=" << it->second;
      }
      ROS_DEBUG_STREAM( dbg.str() );

      rate.sleep();
    }
  }
}


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
