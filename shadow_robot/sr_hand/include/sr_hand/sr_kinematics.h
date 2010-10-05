/**
 * @file   sr_kinematics.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jul 15 12:15:30 2010
 * 
 * @brief  
 * 
 * 
 */

#ifndef _SR_KINEMATICS_H_
#define _SR_KINEMATICS_H_

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <sensor_msgs/JointState.h>

#include "kinematics_msgs/GetPositionIK.h"
#include "kinematics_msgs/PositionIKRequest.h"
#include "motion_planning_msgs/RobotState.h"

namespace shadowrobot
{
class SrKinematics
{
public:
    SrKinematics();
    ~SrKinematics();

private:
    ros::NodeHandle node, n_tilde;

    typedef std::map<std::string, double> JointsMap;
    JointsMap joints_map;
    boost::mutex mutex;

    //static const std::vector<const std::string> joint_names;
    std::vector<std::string> joint_names;

    ros::Publisher pub_hand;
    ros::Publisher pub_arm;
    //we need two different subscribers to joint_states: one for the hand, one for the arm
    ros::Subscriber hand_subscriber;
    ros::Subscriber arm_subscriber;

    //we can have the same callback for both subscribers
    void jointstatesCallback( const sensor_msgs::JointStateConstPtr& msg );

    /**
     * process the reverse kinematics: uses the
     * robot_description parameter (containing the urdf description of the
     * hand) to compute the reverse kinematics.
     *
     * @todo Not yet implemented
     *
     * @param msg a tf transform message
     */
    void reverseKinematicsCallback( const tf::tfMessageConstPtr& msg );
    ///The subscriber to the reverse_kinematics topic
    ros::Subscriber reverse_kinematics_sub;
    tf::TransformListener tf_listener;

    ros::ServiceClient rk_client;

    /**
     * Convert an angle in degree to an angle in radians.
     * @param deg the angle in degrees
     * @return the value in rads.
     */
    inline double toRad( double deg )
    {
        return deg * 3.14159265 / 180.0;
    }
}; // end class SrKinematics

}
; //end namespace


#endif

