/**
 * @file   sr_kinematics.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jul 15 11:54:22 2010
 * 
 * @brief  A ROS node used to subscribe to the tf topic on which the reverse kinematics targets are published.
 * Those targets are then transformed into a pose and sent to the arm_kinematics node which tries to compute
 * the reverse kinematics. On success, the computed targets are then sent to the arm and to the hand.
 * 
 * 
 */

#include <sstream>
#include <algorithm>

#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>

#include <sr_hand/sr_kinematics.h>
#include <sr_hand/sendupdate.h>
#include <sr_hand/joint.h>

using namespace ros;

namespace shadowrobot
{
const std::string SrKinematics::hand_joint_states_topic = "/srh/position/joint_states";
const std::string SrKinematics::arm_joint_states_topic = "/sr_arm/position/joint_states";
const std::string SrKinematics::hand_sendupdate_topic = "/srh/sendupdate";
const std::string SrKinematics::arm_sendupdate_topic = "/sr_arm/sendupdate";

SrKinematics::SrKinematics() :
    n_tilde("~")
{
    /**
     * init the connection to the arm kinematics inverse kinematics service
     */
    if( !n_tilde.getParam("ik_service", arm_kinematics_service) )
    {
        ROS_FATAL("No service name for reverse kinematics found on parameter server");
        return;
    }
    rk_client = node.serviceClient<kinematics_msgs::GetPositionIK> (arm_kinematics_service);

    /**
     * init the publisher on the topic /srh/sendupdate
     * publishing messages of the type sr_hand::sendupdate.
     */
    pub_hand = node.advertise<sr_hand::sendupdate> (hand_sendupdate_topic, 2);
    pub_arm = node.advertise<sr_hand::sendupdate> (arm_sendupdate_topic, 2);

    // getting the root and the tip name from the parameter server
    if( !n_tilde.getParam("root_name", root_name) )
    {
        ROS_FATAL("No root name found on parameter server");
        return;
    }
    if( !n_tilde.getParam("tip_name", tip_name) )
    {
        ROS_FATAL("No tip name found on parameter server");
        return;
    }
    if( !n_tilde.getParam("rk_target", rk_target) )
    {
        ROS_FATAL("No reverse kinematic target found on parameter server");
        return;
    }
    if( !n_tilde.getParam("fixed_frame", fixed_frame) )
    {
        ROS_FATAL("No fixed_frame name found on parameter server");
        return;
    }
    ROS_INFO("Kinematic chain from %s to %s. Reverse kinematics Target: %s", root_name.c_str(), tip_name.c_str(), rk_target.c_str());

    // get the model from the parameter server
    std::string full_param_name, result;
    node.searchParam("urdf_description", full_param_name);
    n_tilde.param("urdf_description", result, std::string("Failed"));

    // Load and Read Models
    if( !loadModel(result) )
    {
        ROS_FATAL("Could not load models!");
        return;
    }

    //subscribe to both the arm and hand joint_states to get all the joints angles.
    std::string full_topic = hand_joint_states_topic;
    hand_subscriber = node.subscribe(full_topic, 10, &SrKinematics::jointstatesCallback, this);
    full_topic = arm_joint_states_topic;
    arm_subscriber = node.subscribe(full_topic, 10, &SrKinematics::jointstatesCallback, this);

    ros::Rate rate = ros::Rate(0.5);
    rate.sleep();
    full_topic = "/tf";
    tf_sub = node.subscribe(full_topic, 10, &SrKinematics::reverseKinematicsCallback, this);
}

SrKinematics::~SrKinematics()
{
}

bool SrKinematics::loadModel( const std::string xml )
{
    urdf::Model robot_model;
    KDL::Tree tree;

    //init the robot_model from the urdf
    if( !robot_model.initString(xml) )
    {
        ROS_FATAL("Could not initialize robot model");
        return -1;
    }

    // build the kinematic chain from tip to root.
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint;
    while( link && link->name != root_name )
    {
        joint = robot_model.getJoint(link->parent_joint->name);
        if( !joint )
        {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return false;
        }

        //add the joint which is in the kinematic chain to the joint map
        joints_map[joint->name.c_str()] = 0.0;
        ROS_INFO( "adding joint: [%s]", joint->name.c_str() );

        //get the parent link
        link = robot_model.getLink(link->getParent()->name);
    }
    return true;
}

void SrKinematics::jointstatesCallback( const sensor_msgs::JointStateConstPtr& msg )
{
    mutex.lock();
    JointsMap::iterator result;

    //update only the joints in the kinematic chain
    for( unsigned int index = 0; index < msg->name.size(); ++index )
    {
        result = joints_map.find(msg->name[index]);
        if( result != joints_map.end() )
        {
            joints_map[msg->name[index]] = msg->position[index];
            continue;
        }
    }
    mutex.unlock();
}

void SrKinematics::reverseKinematicsCallback( const tf::tfMessageConstPtr& msg )
{
    tf::StampedTransform transform;
    std::string link_name = msg->transforms[0].child_frame_id;
    try
    {
        //Only compute the reverse kinematics when receiving something from the kinematics target.
        if( boost::find_first(link_name, rk_target) )
        {
            //the target is publishing the transform between the hand support and the target
            // => we need to get the transform from the origin to the kinematics target
            // this way the transform and the pose are equal.
            tf_listener.lookupTransform(fixed_frame, rk_target, ros::Time(0), transform);

            kinematics_msgs::GetPositionIK srv;
            srv.request.ik_request.ik_link_name = rk_target;
            srv.request.ik_request.pose_stamped = getPoseFromTransform(transform);
            srv.request.timeout = ros::Duration(0.1);

            mutex.lock();
            //Fill the vector with the current joint positions (only those in the kinematics
            // chain).
            for( JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it )
            {
                ROS_DEBUG("pos[%s]: %f", it->first.c_str(), it->second);

                srv.request.ik_request.ik_seed_state.joint_state.name.push_back(it->first);
                srv.request.ik_request.ik_seed_state.joint_state.position.push_back(double(it->second));
            }
            mutex.unlock();

            //Call the reverse kinematics service (implemented in arm_kinematics)
            if( rk_client.call(srv) )
            {
                //The Reverse Kinematics was computed successfully.
                // -> send the computed targets to the joints.
                sr_hand::sendupdate msg;
                std::vector<sr_hand::joint> jointVector;

                for( unsigned int i = 0; i < srv.response.solution.joint_state.name.size(); ++i )
                {
                    std::string sensor_name = srv.response.solution.joint_state.name[i];
                    double target = toDegrees(srv.response.solution.joint_state.position[i]);
                    ROS_DEBUG("[%s] = %f", sensor_name.c_str(), target);

                    //fill the sendupdate message
                    sr_hand::joint joint;
                    joint.joint_name = sensor_name;
                    joint.joint_target = target;
                    jointVector.push_back(joint);
                }

                msg.sendupdate_length = jointVector.size();
                msg.sendupdate_list = jointVector;

                //publish the message for both the hand and the arm.
                pub_hand.publish(msg);
                pub_arm.publish(msg);
            }
            else
            {
                ROS_ERROR("cant compute reverse kinematics");
            }
        }
    }
    catch( tf::TransformException ex )
    {
        ROS_WARN("%s",ex.what());
    }
}

geometry_msgs::PoseStamped SrKinematics::getPoseFromTransform( tf::StampedTransform transform )
{
    geometry_msgs::PoseStamped pose;

    // the transform and the pose are equal, as we're getting the transform from the origin.
    pose.header.stamp = ros::Time::now();
    tf::Vector3 position = transform.getOrigin();
    pose.pose.position.x = double(position.x());
    pose.pose.position.y = double(position.y());
    pose.pose.position.z = double(position.z());
    btQuaternion orientation = transform.getRotation();
    pose.pose.orientation.w = double(orientation.w());
    pose.pose.orientation.x = double(orientation.x());
    pose.pose.orientation.y = double(orientation.y());
    pose.pose.orientation.z = double(orientation.z());

    return pose;
}

}
; //end namespace

/**
 * The main function initializes this reverse kinematics ros node and starts the ros spin loop.
 *
 * @param argc
 * @param argv
 * @return
 */
int main( int argc, char** argv )
{
    ros::init(argc, argv, "sr_kinematics");
    boost::shared_ptr<shadowrobot::SrKinematics> sr_kin(new shadowrobot::SrKinematics::SrKinematics());
    ros::spin();
}

