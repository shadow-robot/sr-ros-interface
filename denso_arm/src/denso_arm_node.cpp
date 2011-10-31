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
  DensoArmNode::DensoArmNode()
    : node_("~")
  {
    denso_arm_ = boost::shared_ptr<DensoArm> ( new DensoArm() );

    init_joints();

    //TODO: read from param
    ros::Rate rate(100);

    //init what's needed for the joint states publishing
    publisher_js_ = node_.advertise<sensor_msgs::JointState>("joint_states", 5);
    timer_joint_states_ = node_.createTimer( rate.expectedCycleTime(), &DensoArmNode::update_joint_states_callback, this);

    //init the actionlib server
    move_arm_pose_server_.reset(new MoveArmPoseServer(node_, "move_arm_pose",
                                                      boost::bind(&DensoArmNode::go_to_arm_pose, this, _1),
                                                      false));

    move_arm_pose_server_->start();
  }

  DensoArmNode::~DensoArmNode()
  {}

  void DensoArmNode::update_joint_states_callback(const ros::TimerEvent& e)
  {
    denso_arm_->update_state( denso_joints_ );

    for( unsigned short index_joint = 0; index_joint < denso_arm_->get_nb_joints() ; ++index_joint)
    {
      joint_state_msg_.position[index_joint] = denso_joints_->at(index_joint).position;
      joint_state_msg_.velocity[index_joint] = denso_joints_->at(index_joint).velocity;
      joint_state_msg_.effort[index_joint] = denso_joints_->at(index_joint).effort;
    }

    publisher_js_.publish( joint_state_msg_ );
  }

  void DensoArmNode::init_joints()
  {
    const unsigned short nb_joints = denso_arm_->get_nb_joints();
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

  void DensoArmNode::go_to_arm_pose(const denso_msgs::MoveArmPoseGoalConstPtr& goal)
  {
    ros::Rate move_arm_rate( goal->rate );
    while( node_.ok() )
    {
      if( move_arm_pose_server_->isPreemptRequested() )
      {

      }
      //TODO: implement me
      move_arm_pose_server_->setSucceeded();
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
