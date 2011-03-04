/**
 * @file   move_arm_simple_action.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jan 26 10:32:10 2011
 *
 * @brief  
 *
 */

#include "sr_move_arm/move_arm_simple_action.hpp"

#include <sr_robot_msgs/joints_data.h>
#include <sr_robot_msgs/joint.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <planning_environment_msgs/GetStateValidity.h>
#include <planning_environment/util/construct_object.h>
#include <planning_environment_msgs/utils.h>

#include <motion_planning_msgs/convert_messages.h>

namespace shadowrobot
{

  SrMoveArmSimpleAction::SrMoveArmSimpleAction() :
    nh_tilde("~")
  {
    action_server = boost::shared_ptr<actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction> >(new actionlib::SimpleActionServer<move_arm_msgs::MoveArmAction>("/right_arm/move_arm", boost::bind(&SrMoveArmSimpleAction::execute, this, _1), false));

    action_server_joint_trajectory = boost::shared_ptr<actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> >(new actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction>(nh, "right_arm/joint_trajectory", boost::bind(&SrMoveArmSimpleAction::execute_trajectory, this, _1), false));

    arm_ik_initialized = false;
    ik_client = nh.serviceClient<kinematics_msgs::GetConstraintAwarePositionIK>("/sr_right_arm_kinematics/get_constraint_aware_ik");
    check_state_validity_client = nh.serviceClient<planning_environment_msgs::GetStateValidity>("get_state_validity");

    sr_arm_target_pub = nh.advertise<sr_robot_msgs::sendupdate>("/sr_arm/sendupdate", 2);
    sr_hand_target_pub = nh.advertise<sr_robot_msgs::sendupdate>("/srh/sendupdate", 2);

    grasp_status_server = nh.advertiseService("right_arm/grasp_status", &SrMoveArmSimpleAction::check_grasp_status, this);

    action_server->start();
    action_server_joint_trajectory->start();
  }

  SrMoveArmSimpleAction::~SrMoveArmSimpleAction()
  {}

  bool SrMoveArmSimpleAction::check_grasp_status(object_manipulation_msgs::GraspStatus::Request &req, object_manipulation_msgs::GraspStatus::Response &res)
  {
    ROS_ERROR("Not implemented yet; check using tactile sensors?");
    res.is_hand_occupied = true;
    return true;
  }

  void SrMoveArmSimpleAction::execute_trajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr& goal)
  {
    if(action_server->isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Execute trajectory preempted.");
      //set action state to preempted
      action_server_joint_trajectory->setPreempted();
    }
      
    //initializes the joint names
    std::vector<std::string> joint_names = goal->trajectory.joint_names;
    joint_vector_traj.clear();
    for(unsigned int i = 0; i < joint_names.size(); ++i)
    {
      sr_robot_msgs::joint joint;
      joint.joint_name = joint_names[i];
      joint_vector_traj.push_back(joint);
    }
    sendupdate_msg_traj.sendupdate_length = joint_vector_traj.size();

    ROS_DEBUG("Trajectory received: %d joints / %d msg length", (int)goal->trajectory.joint_names.size(), sendupdate_msg_traj.sendupdate_length);

    
    std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = goal->trajectory.points;
    trajectory_msgs::JointTrajectoryPoint trajectory_step;
    
    //loop through the steps
    for(unsigned int index_step = 0; index_step < trajectory_points.size(); ++index_step)
    {
      trajectory_step = trajectory_points[index_step];

      //update the targets
      for(unsigned int index_pos = 0; index_pos < (unsigned int)sendupdate_msg_traj.sendupdate_length; ++index_pos)
      {
        joint_vector_traj[index_pos].joint_target = math_utils.to_degrees(trajectory_step.positions[index_pos]);

        ROS_DEBUG("traj[%s]: %f", joint_vector_traj[index_pos].joint_name.c_str(), joint_vector_traj[index_pos].joint_target);

      }
      sendupdate_msg_traj.sendupdate_list = joint_vector_traj;
      
      sr_arm_target_pub.publish(sendupdate_msg_traj);
      sr_hand_target_pub.publish(sendupdate_msg_traj);
      
      trajectory_step.time_from_start.sleep();
      ROS_DEBUG("Step %d of %d done.", index_step + 1, (int)trajectory_points.size());

      ROS_DEBUG("End step----");
    }

    action_server_joint_trajectory->setSucceeded(joint_trajectory_result);
  }

  void SrMoveArmSimpleAction::execute(const move_arm_msgs::MoveArmGoalConstPtr& goal)
  {
    ros::Rate ts(1.0);
    move_arm_action_feedback.time_to_completion = ts.cycleTime();
    move_arm_action_feedback.state = "stopped";
    action_server->publishFeedback(move_arm_action_feedback);
    
    bool arm_in_correct_position = true;

    if(action_server->isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Move arm action preempted.");
      //set action state to preempted
      action_server->setPreempted();
      arm_in_correct_position = false;
    }
      
    //TODO: compute an approximate remaining time
    move_arm_action_feedback.time_to_completion = ros::Rate(1.0).cycleTime();
    move_arm_action_feedback.state = "moving";
    action_server->publishFeedback(move_arm_action_feedback);
    
    joint_vector.clear();


    //if the joint path is empty
    if( goal->motion_plan_request.goal_constraints.joint_constraints.size() == 0 )
    { 
      //received a pose target=> compute the ik and move the arm to the given target if possible
      if( goal->motion_plan_request.goal_constraints.position_constraints.size() != 0 )
      {
        ROS_INFO("Planning to a pose goal");
        
        motion_planning_msgs::MotionPlanRequest req;
        req.workspace_parameters   = goal->motion_plan_request.workspace_parameters;
        req.start_state            = goal->motion_plan_request.start_state;
        req.goal_constraints       = goal->motion_plan_request.goal_constraints;
        req.path_constraints       = goal->motion_plan_request.path_constraints;
        req.allowed_contacts       = goal->motion_plan_request.allowed_contacts;
        req.ordered_collision_operations = goal->motion_plan_request.ordered_collision_operations;
        req.link_padding           = goal->motion_plan_request.link_padding;
        req.planner_id             = goal->motion_plan_request.planner_id;
        req.group_name             = goal->motion_plan_request.group_name;
        req.num_planning_attempts  = goal->motion_plan_request.num_planning_attempts;
        req.allowed_planning_time  = goal->motion_plan_request.allowed_planning_time;
        req.expected_path_duration = goal->motion_plan_request.expected_path_duration;
        req.expected_path_dt       = goal->motion_plan_request.expected_path_dt;

        
        if(!convertPoseGoalToJointGoal(req))
        {
          action_server->setAborted( move_arm_action_result );
          return;
        }
      }

      //no constraints at all=>bad goal received
      else
      {
        move_arm_action_feedback.time_to_completion = ros::Rate(0.0).cycleTime();
        move_arm_action_feedback.state = "stopped";
        action_server->publishFeedback(move_arm_action_feedback);
        move_arm_action_result.error_code.val = -13;
        action_server->setAborted( move_arm_action_result );
        return;
      }
    }
    //received a joint trajectory: move the arm through the trajectory
    else
    {
      for( unsigned int i=0; i < goal->motion_plan_request.goal_constraints.joint_constraints.size(); ++i )
      {
        sr_robot_msgs::joint joint;
        joint.joint_name = goal->motion_plan_request.goal_constraints.joint_constraints[i].joint_name;
        joint.joint_target = math_utils.to_degrees(goal->motion_plan_request.goal_constraints.joint_constraints[i].position);
        
        joint_vector.push_back(joint);
      }
    }
     
    sendupdate_msg.sendupdate_length = joint_vector.size();
    sendupdate_msg.sendupdate_list = joint_vector;
    
    sr_arm_target_pub.publish(sendupdate_msg);
    sr_hand_target_pub.publish(sendupdate_msg);

    arm_in_correct_position = true;
    
    if(arm_in_correct_position)
    {
      move_arm_action_feedback.time_to_completion = ros::Rate(0.0).cycleTime();
      move_arm_action_feedback.state = "stopped";
      action_server->publishFeedback(move_arm_action_feedback);
      move_arm_action_result.error_code.val = 0;
      action_server->setSucceeded(move_arm_action_result);
    }

    ROS_DEBUG("Arm movement done");
  }

  bool SrMoveArmSimpleAction::convertPoseGoalToJointGoal(motion_planning_msgs::MotionPlanRequest &req)
  {
    if(!arm_ik_initialized)
    {
      if(!ros::service::waitForService("/sr_right_arm_kinematics/get_constraint_aware_ik",ros::Duration(1.0)))
      {
        ROS_WARN("Inverse kinematics service is unavailable");
        return false;
      }
      else
      {
        arm_ik_initialized = true;
      }
    }


    ROS_DEBUG("Acting on goal to pose ...");// we do IK to find corresponding states
    ROS_DEBUG("Position constraint: %f %f %f",
              req.goal_constraints.position_constraints[0].position.x,
              req.goal_constraints.position_constraints[0].position.y,
              req.goal_constraints.position_constraints[0].position.z);
    ROS_DEBUG("Orientation constraint: %f %f %f %f",
              req.goal_constraints.orientation_constraints[0].orientation.x,
              req.goal_constraints.orientation_constraints[0].orientation.y,
              req.goal_constraints.orientation_constraints[0].orientation.z,
              req.goal_constraints.orientation_constraints[0].orientation.w);

    geometry_msgs::PoseStamped tpose = motion_planning_msgs::poseConstraintsToPoseStamped(req.goal_constraints.position_constraints[0],
                                                                                          req.goal_constraints.orientation_constraints[0]);
    std::string link_name = req.goal_constraints.position_constraints[0].link_name;
    sensor_msgs::JointState solution;		

    ROS_INFO("IK request");
    ROS_INFO("link_name   : %s",tpose.header.frame_id.c_str());
    ROS_INFO("frame_id    : %s",tpose.header.frame_id.c_str());
    ROS_INFO("position    : (%f,%f,%f)",tpose.pose.position.x,tpose.pose.position.y,tpose.pose.position.z);
    ROS_INFO("orientation : (%f,%f,%f,%f)",tpose.pose.orientation.x,tpose.pose.orientation.y,tpose.pose.orientation.z,tpose.pose.orientation.w);
    ROS_INFO(" ");

    if (computeIK(tpose, link_name, solution))
    {
      /*if(!checkIK(tpose,link_name,solution))
        ROS_ERROR("IK solution does not get to desired pose");
      */
      motion_planning_msgs::RobotState ik_sanity_check;
      ik_sanity_check.joint_state.header = tpose.header;

      for (unsigned int i = 0 ; i < solution.name.size() ; ++i)
      {
        motion_planning_msgs::JointConstraint jc;
        jc.joint_name = solution.name[i];
        jc.position = solution.position[i];
        jc.tolerance_below = 0.01;
        jc.tolerance_above = 0.01;
        req.goal_constraints.joint_constraints.push_back(jc);
        ik_sanity_check.joint_state.name.push_back(jc.joint_name);
        ik_sanity_check.joint_state.position.push_back(jc.position);
      }
      motion_planning_msgs::ArmNavigationErrorCodes error_code;
      if(!isStateValidAtGoal(ik_sanity_check, error_code))
      {
        ROS_INFO("IK returned joint state for goal that doesn't seem to be valid");
        if(error_code.val == error_code.GOAL_CONSTRAINTS_VIOLATED) {
          ROS_WARN("IK solution doesn't obey goal constraints");
        } else if(error_code.val == error_code.COLLISION_CONSTRAINTS_VIOLATED) {
          ROS_WARN("IK solution in collision");
        } else {
          ROS_WARN_STREAM("Some other problem with ik solution " << error_code.val);
        }
      }
      req.goal_constraints.position_constraints.clear();
      req.goal_constraints.orientation_constraints.clear();	    
      return true;
    }
    else
      return false;
  }


  bool SrMoveArmSimpleAction::isStateValidAtGoal(const motion_planning_msgs::RobotState& state,
                                                 motion_planning_msgs::ArmNavigationErrorCodes& error_code)
  {
    planning_environment_msgs::GetStateValidity::Request req;
    planning_environment_msgs::GetStateValidity::Response res;
    req.robot_state = state;
    req.check_goal_constraints = true;
    req.check_collisions = true;
    planning_environment_msgs::generateDisableAllowedCollisionsWithExclusions(all_link_names_,
                                                                              group_link_names_,
                                                                              req.ordered_collision_operations.collision_operations);
    req.ordered_collision_operations.collision_operations.insert(req.ordered_collision_operations.collision_operations.end(),
                                                                 original_request_.motion_plan_request.ordered_collision_operations.collision_operations.begin(),
                                                                 original_request_.motion_plan_request.ordered_collision_operations.collision_operations.end());
    req.allowed_contacts = original_request_.motion_plan_request.allowed_contacts;
    req.path_constraints = original_request_.motion_plan_request.path_constraints;
    req.goal_constraints = original_request_.motion_plan_request.goal_constraints;
    req.link_padding = original_request_.motion_plan_request.link_padding;
    if(check_state_validity_client.call(req,res))
    {
      error_code = res.error_code;
      if(res.error_code.val == res.error_code.SUCCESS)
        return true;
      else
        return false;
    }
    else
    {
      ROS_ERROR("Service call to check goal validity failed %s",
                check_state_validity_client.getService().c_str());
      return false;
    }
  }

  bool SrMoveArmSimpleAction::computeIK(const geometry_msgs::PoseStamped &pose_stamped_msg,  
                                        const std::string &link_name, 
                                        sensor_msgs::JointState &solution)
  {
    kinematics_msgs::GetConstraintAwarePositionIK::Request request;
    kinematics_msgs::GetConstraintAwarePositionIK::Response response;
	    
    request.ik_request.pose_stamped = pose_stamped_msg;
    if(!getRobotState(request.ik_request.robot_state)) {
      return false;
    }
    request.ik_request.ik_seed_state = request.ik_request.robot_state;
    request.ik_request.ik_link_name = link_name;
    request.timeout = ros::Duration(ik_allowed_time_);
    request.ordered_collision_operations = original_request_.motion_plan_request.ordered_collision_operations;
    request.allowed_contacts = original_request_.motion_plan_request.allowed_contacts;
    request.constraints = original_request_.motion_plan_request.goal_constraints;
    request.link_padding = original_request_.motion_plan_request.link_padding;
    if (ik_client.call(request, response))
    {
      move_arm_action_result.error_code = response.error_code;
      if(response.error_code.val != response.error_code.SUCCESS)
      {
        ROS_ERROR("IK Solution not found, IK returned with error_code: %d",response.error_code.val);
        return false;
      }         
      solution = response.solution.joint_state;
      if (solution.position.size() != group_joint_names_.size())
      {
        ROS_ERROR("Incorrect number of elements in IK output.");
        return false;
      }
      for(unsigned int i = 0; i < solution.position.size() ; ++i)
        ROS_DEBUG("IK[%d] = %f", (int)i, solution.position[i]);
    }
    else
    {
      ROS_ERROR("IK service failed");
      return false;
    }	    
    return true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_move_arm");

  ros::AsyncSpinner spinner(1); //Use 1 thread
  spinner.start();
  shadowrobot::SrMoveArmSimpleAction move_arm;
  ros::spin();
  //ros::waitForShutdown();

  return 0;
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
