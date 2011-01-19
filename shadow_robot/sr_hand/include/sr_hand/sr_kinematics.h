/**
 * @file   sr_kinematics.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jul 15 12:15:30 2010
 * 
 * @brief  A ROS node used to subscribe to the tf topic on which the reverse kinematics targets are published.
 * Those targets are then transformed into a pose and sent to the arm_kinematics node which tries to compute
 * the reverse kinematics. On success, the computed targets are then sent to the arm and to the hand.
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
#include <tf_conversions/tf_kdl.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

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

    //consts used for the topics on which to publish / subscribe
    static const std::string hand_joint_states_topic, arm_joint_states_topic, hand_sendupdate_topic, arm_sendupdate_topic;

    /**
     * the name of the link composing the root and the one of the tip of the kinematic chain
     */
    std::string arm_kinematics_service, root_name, tip_name, rk_target, fixed_frame;

    /**
     * contains the kinematic chain from root to tip
     */
    KDL::Chain chain;

    typedef std::map<std::string, double> JointsMap;
    /**
     * A map containing the joints in the kinematic chain (from root to tip) and their current positions
     */
    JointsMap joints_map;

    /**
     * Load the robot model and extract the correct joint_names from the
     * kinematic chain, from root to tip.
     * @param xml the xml of the robot model
     * @return true if success
     */
    bool loadModel( const std::string xml );

    boost::mutex mutex;

    //publishes all the targets to the hand and the arm (the target names are then checked internally in the hand/arm)
    //it's easier that way than checking if the target is for the hand or for the arm.
    ros::Publisher pub_hand, pub_arm;
    //we need two different subscribers to joint_states: one for the hand, one for the arm
    ros::Subscriber hand_subscriber, arm_subscriber;

    /**
     * The callback for the joint_states topic. Periodically update the current position
     * for the joints in the kinematic chain.
     *
     * We can have the same callback for both joint_states subscribers:
     * we're updating joints_map with data coming from both the hand and the arm.
     * @param msg a JointState message
     */
    void jointstatesCallback( const sensor_msgs::JointStateConstPtr& msg );

    /**
     * compute the reverse kinematics for the kinematic chain between the specified
     * root and the tip.
     *
     * @param msg a tf transform message
     */
    void reverseKinematicsCallback( const tf::tfMessageConstPtr& msg );
    ///The subscriber to the tf topic
    ros::Subscriber tf_sub;
    ///A transform listener used to get the transform between the reverse kinematics target and the root.
    tf::TransformListener tf_listener;

    ///The client for the reverse kinematics service implemented in arm_kinematics.
    ros::ServiceClient rk_client;

    /**
     * Convert the given transform to a pose.
     *
     * In our case, the transform and the pose are equal as we're taking the transform from the origin.
     *
     * @param transform Transform from the origin to the reverse kinematics target.
     * @return Pose of the kinematics target.
     */
    geometry_msgs::PoseStamped getPoseFromTransform(tf::StampedTransform transform);

    /**
     * Convert an angle in radians to an angle in degrees.
     * @param rad the angle in radians
     * @return the value in degrees.
     */
    inline double toDegrees( double rad )
    {
        return rad * 57.2957795;
    }
}; // end class SrKinematics

}
; //end namespace


#endif

