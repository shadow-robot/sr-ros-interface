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

    find_cluster_bounding_box_client = nh.serviceClient<object_manipulation_msgs::FindClusterBoundingBox>("/find_cluster_bounding_box");

    grasp_server = nh_tilde.advertiseService("plan_point_cluster_grasp",
                                             &SrGraspPlannerServer::get_grasp_planning_callback,
                                             this);
  }

  SrGraspPlannerServer::~SrGraspPlannerServer()
  {}

  bool SrGraspPlannerServer::get_grasp_planning_callback( object_manipulation_msgs::GraspPlanning::Request &request,
                                                          object_manipulation_msgs::GraspPlanning::Response &response )
  {
    ROS_INFO("Planning grasps for %s", request.collision_object_name.c_str());

    object_manipulation_msgs::FindClusterBoundingBox srv;
    object_manipulation_msgs::ClusterBoundingBox bounding_box;
    srv.request.cluster = request.target.cluster;
    ROS_DEBUG_STREAM("Cluster for the unknown object"<<srv.request.cluster);

    if( find_cluster_bounding_box_client.call(srv) == 1)
    {
      ROS_DEBUG_STREAM("Response Pose " << srv.response.pose <<
                       "Response Box Dims " << srv.response.box_dims);

      bounding_box.pose_stamped = srv.response.pose;
      bounding_box.dimensions = srv.response.box_dims;
    }
    else
    {
      ROS_ERROR("Failed to get the bounding box for the unknown object");
      response.error_code.value = response.error_code.OTHER_ERROR;
      return false;
    }

    response.grasps = grasp_planner->compute_list_of_grasps(bounding_box);

    if( response.grasps.size() != 0)
    {
      response.error_code.value = response.error_code.SUCCESS;
      return true;
    }
    else
    {
      response.error_code.value = response.error_code.OTHER_ERROR;
      return false;
    }
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
