/**
 * @file   sr_grasp_planner_server.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Mar 22 10:37:43 2011
 *
 * @brief  The Grasp Planner server: provides a
 * ros service to call the grasp planner.
 *
 *
 */

#include "sr_grasp_planner/sr_grasp_planner_server.hpp"

namespace shadowrobot
{
  SrGraspPlannerServer::SrGraspPlannerServer() :
    nh_tilde("~")
  {
    grasp_planner = boost::shared_ptr<SrGraspPlanner>(new SrGraspPlanner());

    grasp_server = nh_tilde.advertiseService("plan_point_cluster_grasp",
                                             &SrGraspPlannerServer::get_grasp_planning_callback,
                                             this);
  }

  SrGraspPlannerServer::~SrGraspPlannerServer()
  {}

  bool SrGraspPlannerServer::get_grasp_planning_callback( object_manipulation_msgs::GraspPlanning::Request &request,
                                                          object_manipulation_msgs::GraspPlanning::Response &response )
  {
    ROS_ERROR("Implement the callback");
    tf::StampedTransform transform;
    try{
      tf_listener.lookupTransform("/base_link", "/palm",
                                  ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    geometry_msgs::Pose current_pose;
    current_pose.position.x = transform.getOrigin().getX();
    current_pose.position.y = transform.getOrigin().getY();
    current_pose.position.z = transform.getOrigin().getZ();
    current_pose.orientation.x = transform.getRotation().getX();
    current_pose.orientation.y = transform.getRotation().getY();
    current_pose.orientation.z = transform.getRotation().getZ();
    current_pose.orientation.w = transform.getRotation().getW();

    response.grasps = grasp_planner->compute_list_of_grasps(request.target, current_pose);

    response.error_code.value = response.error_code.SUCCESS;
    return true;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_grasp_planner");

  shadowrobot::SrGraspPlannerServer grasp_planner_server;
  ros::spin();
  //ros::waitForShutdown();

  return 0;
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
