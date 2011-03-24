/**
 * @file   sr_grasp_planner_server.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Mar 22 10:43:37 2011
 *
 * @brief  The Grasp Planner server: provides a
 * ros service to call the grasp planner.
 *
 *
 */

#ifndef _SR_GRASP_PLANNER_SERVER_H_
#define _SR_GRASP_PLANNER_SERVER_H_

#include <ros/ros.h>

#include "sr_grasp_planner/sr_grasp_planner.hpp"
#include <object_manipulation_msgs/GraspPlanning.h>
#include <object_manipulation_msgs/FindClusterBoundingBox.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/smart_ptr.hpp>

namespace shadowrobot
{
  class SrGraspPlannerServer
  {
  public:
    SrGraspPlannerServer();
    ~SrGraspPlannerServer();

  protected:
    ros::NodeHandle nh, nh_tilde;

    /** a grasp planner, used to compute the possible grasps */
    boost::shared_ptr<SrGraspPlanner> grasp_planner;
    /**
     * the ros service server calling the grasp planner and returning
     * the possible grasps
     */
    ros::ServiceServer grasp_server;

    /** A client used to get the bounding box for the cluster */
    ros::ServiceClient find_cluster_bounding_box_client;

    /**
     * The grasp planning callback for the grasp_server.
     *
     * @param request The request contains information about the object to grasp
     * @param response A list of possible grasps.
     *
     * @return true if success.
     */
    bool get_grasp_planning_callback( object_manipulation_msgs::GraspPlanning::Request &request,
                                      object_manipulation_msgs::GraspPlanning::Response &response );

    /** A transform listener to get the hand pose */
    tf::TransformListener tf_listener;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
