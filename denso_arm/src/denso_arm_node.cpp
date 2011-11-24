/**
 * @file   denso_arm_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Dan Greenwald <dg@shadowrobot.com>
 * @date   Mon Oct 31 09:26:15 2011
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
 * @brief A standard interface to the DENSO arm.
 *
 *
 */

#include "denso_arm/denso_arm_node.hpp"

namespace denso
{
  const bool DensoArmNode::simulated = false;

  DensoArmNode::DensoArmNode()
    : node_("~")
  {
    if( !simulated )
      denso_arm_ = boost::shared_ptr<DensoArm> ( new DensoArm() );

    init_joints();

    //TODO: read from param
    ros::Rate rate_js(50);
    ros::Rate rate_tip(35);

    //init what's needed for the joint states publishing
    publisher_js_ = node_.advertise<sensor_msgs::JointState>("joint_states", 5);
    timer_joint_states_ = node_.createTimer( rate_js.expectedCycleTime(), &DensoArmNode::update_joint_states_callback, this);

    //init the publishing of the tip pose
    tip_pose_ = boost::shared_ptr<Pose>( new Pose() );
    timer_tip_pose_ = node_.createTimer( rate_tip.expectedCycleTime(), &DensoArmNode::update_tip_pose_callback, this);
    tf_tip_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>( new tf::TransformBroadcaster() );

    //init the tooltip server
    tooltip_server = node_.advertiseService("set_tooltip", &DensoArmNode::set_tooltip, this);

    //init the actionlib server
    move_arm_pose_server_.reset(new MoveArmPoseServer(node_, "move_arm_pose",
                                                      boost::bind(&DensoArmNode::go_to_arm_pose, this, _1),
                                                      false));

    move_arm_pose_server_->start();
  }

  DensoArmNode::~DensoArmNode()
  {}

  void DensoArmNode::update_tip_pose_callback(const ros::TimerEvent& e)
  {
    if( !denso_mutex.try_lock() )
      return;

    denso_arm_->get_cartesian_position( tip_pose_ );

    denso_mutex.unlock();

    tf_tip_broadcaster_->sendTransform( tf::StampedTransform( denso_pose_to_tf_transform(tip_pose_),
                                                              ros::Time::now(),
                                                              "/denso_arm/base",
                                                              "/denso_arm/tooltip"));
  }

  void DensoArmNode::update_joint_states_callback(const ros::TimerEvent& e)
  {
    if( !denso_mutex.try_lock() )
      return;

    if( !simulated )
      denso_arm_->update_state( denso_joints_ );

    denso_mutex.unlock();

    joint_state_msg_.header.stamp = ros::Time::now();
    if( !simulated )
    {
      for( unsigned short index_joint = 0; index_joint < denso_arm_->get_nb_joints() ; ++index_joint)
      {
        joint_state_msg_.position[index_joint] = denso_joints_->at(index_joint).position;
        joint_state_msg_.velocity[index_joint] = denso_joints_->at(index_joint).velocity;
        joint_state_msg_.effort[index_joint] = denso_joints_->at(index_joint).effort;
      }
    }

    publisher_js_.publish( joint_state_msg_ );
  }

  void DensoArmNode::init_joints()
  {
    unsigned short nb_joints = 6;
    if( !simulated )
      nb_joints = denso_arm_->get_nb_joints();
    denso_joints_ = boost::shared_ptr<DensoJointsVector>( new DensoJointsVector( nb_joints ) );

    for( unsigned short i=0; i < nb_joints; ++i )
    {
      std::stringstream ss;
      ss << "denso_J" << i;
      denso_joints_->at(i).name = ss.str();

      joint_state_msg_.name.push_back( ss.str() );

      joint_state_msg_.position.push_back( 0.0 );
      joint_state_msg_.velocity.push_back( 0.0 );
      joint_state_msg_.effort.push_back( 0.0 );
    }
  }


  bool DensoArmNode::set_tooltip( denso_msgs::SetTooltip::Request& req,
                                  denso_msgs::SetTooltip::Response& resp )
  {
    return denso_arm_->set_tooltip( req.tool_number );
  }

  void DensoArmNode::go_to_arm_pose(const denso_msgs::MoveArmPoseGoalConstPtr& goal)
  {
    int success = denso_msgs::MoveArmPoseResult::SUCCESS;

    //rate at which we'll send the demands to the arm
    ros::Rate move_arm_rate( goal->rate );

    //for checking the time out
    ros::Duration time_left( goal->time_out );
    ros::Time start_time = ros::Time::now();

    while( node_.ok() )
    {
      if( move_arm_pose_server_->isPreemptRequested() || !ros::ok() )
      {
        ROS_INFO("Move Denso arm to pose preempted.");
        move_arm_pose_server_->setPreempted();
        success = denso_msgs::MoveArmPoseResult::PREEMPTED;
        break;
      }

      //Compute pose from goal
      geometry_msgs::Pose pose_tmp( goal->goal );
      Pose pose_goal = geometry_pose_to_denso_pose( pose_tmp );

      ROS_DEBUG_STREAM(" RPY : " << pose_goal.roll << " / " << pose_goal.pitch << " / " << pose_goal.yaw);

      bool locked = false;
      for( unsigned int i=0; i < 1000; ++i)
      {
        if( denso_mutex.try_lock() )
        {
          locked = true;
          break;
        }
        usleep(10000);
      }

      if( locked )
      {
        if( !simulated )
        {
	  denso_arm_->set_speed( goal->speed );

          if( denso_arm_->send_cartesian_position( pose_goal ) )
          {
            denso_mutex.unlock();
            break; //We reached the target -> SUCCESS
          }
        }
        else
        {
          usleep( 500000 );
          denso_mutex.unlock();
          break;
        }
      }
      else
      {
        ROS_WARN("Couldn't send the target to the arm.");
      }

      //publish feedback
      time_left -= (ros::Time::now() - start_time);
      move_arm_pose_feedback_.time_left = time_left;
      move_arm_pose_server_->publishFeedback( move_arm_pose_feedback_ );

      //check if timedout
      if( time_left.toSec() <= 0.0 )
      {
        success = denso_msgs::MoveArmPoseResult::TIMED_OUT;
        break;
      }

      //loop at the rate the user asked for
      move_arm_rate.sleep();
    }

    if( success == denso_msgs::MoveArmPoseResult::SUCCESS)
    {
      move_arm_pose_result_.val = denso_msgs::MoveArmPoseResult::SUCCESS;
      move_arm_pose_server_->setSucceeded( move_arm_pose_result_ );
    }
    else
    {
      //failed
      move_arm_pose_result_.val = success;
      move_arm_pose_server_->setAborted( move_arm_pose_result_ );
    }
  }
}

int main(int argc, char *argv[])
{
  ros::init( argc, argv, "denso_arm" );

  boost::shared_ptr<denso::DensoArmNode> denso_node;
  denso_node = boost::shared_ptr<denso::DensoArmNode> ( new denso::DensoArmNode() );

  ros::spin();

  return 0;
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
