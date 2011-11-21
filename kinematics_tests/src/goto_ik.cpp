#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

#include <map>

#include <std_msgs/Float64.h>
#include <pr2_mechanism_msgs/ListControllers.h>


std::map<std::string,std::string> jointControllerMap;   
std::map<std::string,unsigned int> jointPubIdxMap;
ros::Publisher pub[4];
ros::ServiceClient ik_client;
kinematics_msgs::GetPositionIK::Request  gpik_req;
kinematics_msgs::GetPositionIK::Response gpik_res;

kinematics_msgs::GetKinematicSolverInfo::Request request;
kinematics_msgs::GetKinematicSolverInfo::Response response;


// get IK for x,y,z and send joint pos to joint pos controllers
int moveIK(float x, float y, float z)
{
	gpik_req.ik_request.pose_stamped.pose.position.x = x;
  gpik_req.ik_request.pose_stamped.pose.position.y = y;
  gpik_req.ik_request.pose_stamped.pose.position.z = z;

/*	//seed is not relevant with the used ik_solver but set it anyway
  for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
  {
    gpik_req.ik_request.ik_seed_state.joint_state.position[i] = 0.0;
  }*/
  if(ik_client.call(gpik_req, gpik_res))
  {
    if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
		{
		  
		  std_msgs::Float64 message;
			for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
			{
		  	ROS_DEBUG("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
				ROS_DEBUG("we publish to %s",pub[i].getTopic().c_str());
				message.data= (double)gpik_res.solution.joint_state.position[i];
				pub[jointPubIdxMap[gpik_res.solution.joint_state.name[i]]].publish(message);
			}
      ros::spinOnce();
			return 0;
		}
    else
      ROS_ERROR("Inverse kinematics failed");
			return -1;
  }
  else
    ROS_ERROR("Inverse kinematics service call failed");
		return -1;
}


int main(int argc, char **argv){
  ros::init (argc, argv, "move_ik");
  ros::NodeHandle rh;

	// prepare services
  ros::service::waitForService("ff_kinematics/get_ik_solver_info");
  ros::service::waitForService("ff_kinematics/get_ik");

  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::GetKinematicSolverInfo>("ff_kinematics/get_ik_solver_info");
  ik_client = rh.serviceClient<kinematics_msgs::GetPositionIK>("ff_kinematics/get_ik");

	ros::service::waitForService("pr2_controller_manager/list_controllers");
	ros::ServiceClient controller_list_client = rh.serviceClient<pr2_mechanism_msgs::ListControllers>("pr2_controller_manager/list_controllers");

	// init treated joint (to be modified to get more generic behaviour)
	std::string controlled_joint_name;
	std::vector<std::string> joint_name;
	joint_name.push_back("FFJ1");
	joint_name.push_back("FFJ2");
	joint_name.push_back("FFJ3");
	joint_name.push_back("FFJ4");

	// init jointControllerMapping
	pr2_mechanism_msgs::ListControllers controller_list;
  controller_list_client.call(controller_list);
  for (unsigned int i=0;i<controller_list.response.controllers.size() ;i++ )
  {
   		if (rh.getParam("/"+controller_list.response.controllers[i]+"/joint", controlled_joint_name))
			{
				ROS_DEBUG("controller %d:%s controlls joint %s\n",i,controller_list.response.controllers[i].c_str(),controlled_joint_name.c_str());
				jointControllerMap[controlled_joint_name]= controller_list.response.controllers[i] ;
			}
  }

  // get possible computable joints
  if(query_client.call(request,response))
  {
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
    }
  }
  else
  {
    ROS_ERROR("Could not call query service");
    ros::shutdown();
    exit(1);
  }

	// define the service messages
  gpik_req.timeout = ros::Duration(5.0);
  gpik_req.ik_request.ik_link_name = "ffdistal";
  gpik_req.ik_request.pose_stamped.header.frame_id = "palm";
  gpik_req.ik_request.pose_stamped.pose.position.x = 0.0;
  gpik_req.ik_request.pose_stamped.pose.position.y = 0.0;
  gpik_req.ik_request.pose_stamped.pose.position.z = 0.0;

  //pos is not relevant with the used ik_solver but set it anyway to identity
  gpik_req.ik_request.pose_stamped.pose.orientation.x = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.y = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.z = 0.0;
  gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0;
  gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
  gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
	

	// prepare the publishers
	for(unsigned int i=0; i < joint_name.size(); i ++)
	{
	  pub[i] = rh.advertise<std_msgs::Float64>("/"+jointControllerMap[joint_name[i]]+"/command", 2);
		jointPubIdxMap[joint_name[i]]=i;
	}	
	
	ros::spinOnce();
  sleep(1); // this is required otherwise publishers are not ready for first messages to be sent

  moveIK(0.033,-0.055,0.095+0.075);
	sleep(2);

  ros::shutdown();
}




