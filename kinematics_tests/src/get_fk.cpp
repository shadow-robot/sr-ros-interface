#include <ros/ros.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "get_fk");
  ros::NodeHandle rh;

  ros::service::waitForService("sr_right_arm_kinematics/get_fk_solver_info");
  ros::service::waitForService("sr_right_arm_kinematics/get_fk");

  ros::ServiceClient query_client = 
    rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>
    ("sr_right_arm_kinematics/get_fk_solver_info");
  ros::ServiceClient fk_client = rh.serviceClient
    <kinematics_msgs::GetPositionFK>("sr_right_arm_kinematics/get_fk");

  // define the service messages
  kinematics_msgs::GetKinematicSolverInfo::Request request;
  kinematics_msgs::GetKinematicSolverInfo::Response response;
  if(query_client.call(request,response))
  {
    for(unsigned int i=0; 
        i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s", i,
       response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }
  // define the service messages
  kinematics_msgs::GetPositionFK::Request  fk_request;
  kinematics_msgs::GetPositionFK::Response fk_response;
  fk_request.header.frame_id = "base_link";
  fk_request.fk_link_names.resize(1);
  fk_request.fk_link_names[0] = "palm";

  fk_request.robot_state.joint_state.position.resize
    (response.kinematic_solver_info.joint_names.size());
  fk_request.robot_state.joint_state.name = 
    response.kinematic_solver_info.joint_names;
  for(unsigned int i=0; 
      i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    fk_request.robot_state.joint_state.position[i] = 0.5;
  }
  if(fk_client.call(fk_request, fk_response))
  {
    if(fk_response.error_code.val == fk_response.error_code.SUCCESS)
    {
      for(unsigned int i=0; i < fk_response.pose_stamped.size(); i ++)
      {
        ROS_INFO_STREAM("Link    : " << fk_response.fk_link_names[i].c_str());
        ROS_INFO_STREAM("Position: " << 
          fk_response.pose_stamped[i].pose.position.x << "," <<  
          fk_response.pose_stamped[i].pose.position.y << "," << 
          fk_response.pose_stamped[i].pose.position.z);
        ROS_INFO("Orientation: %f %f %f %f",
          fk_response.pose_stamped[i].pose.orientation.x,
          fk_response.pose_stamped[i].pose.orientation.y,
          fk_response.pose_stamped[i].pose.orientation.z,
          fk_response.pose_stamped[i].pose.orientation.w);
      } 
    }
    else
    {
      ROS_ERROR("Forward kinematics failed");
    }
  }
  else
  {
    ROS_ERROR("Forward kinematics service call failed");
  }
  ros::shutdown();
}

