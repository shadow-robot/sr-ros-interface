/**
 * @file   sr_kinematics.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jul 15 11:54:22 2010
 * 
 * @brief  
 * 
 * 
 */

#include <kdl/jntarray.hpp>
#include <sstream>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>

#include "sr_hand/sr_kinematics.h"
#include <sr_hand/sendupdate.h>
#include <sr_hand/joint.h>
using namespace ros;

namespace shadowrobot
{
//const std::string tmp[8] = {"base_fix", "trunk_rotation", "shoulder_rotation", "elbow_abduction", "forearm_rotation", "arm_link", "WRJ2", "WRJ1"};
//static const std::vector<const std::string> joint_names(tmp, tmp+8);


SrKinematics::SrKinematics()
{

    rk_client = node.serviceClient<kinematics_msgs::GetPositionIK> ("/arm_kinematics/get_ik");

    /**
     * init the publisher on the topic /srh/sendupdate
     * publishing messages of the type sr_hand::sendupdate.
     */
    pub_hand = node.advertise<sr_hand::sendupdate> ("/srh/sendupdate", 2);
    pub_arm = node.advertise<sr_hand::sendupdate> ("/sr_arm/sendupdate", 2);

    //joint_names.push_back("base_fix");
    joint_names.push_back("trunk_rotation");
    joint_names.push_back("shoulder_rotation");
    joint_names.push_back("elbow_abduction");
    joint_names.push_back("forearm_rotation");
    joint_names.push_back("arm_link");
    joint_names.push_back("WRJ2");
    joint_names.push_back("WRJ1");

    //subscribe to both the arm and hand joint_states to get all the joints angles.
    std::string full_topic = "/srh/position/joint_states";
    hand_subscriber = node.subscribe(full_topic, 10, &SrKinematics::jointstatesCallback, this);
    full_topic = "/sr_arm/position/joint_states";
    arm_subscriber = node.subscribe(full_topic, 10, &SrKinematics::jointstatesCallback, this);

    ros::Rate rate = ros::Rate(0.5);
    rate.sleep();
    full_topic = "/tf";
    reverse_kinematics_sub = node.subscribe(full_topic, 10, &SrKinematics::reverseKinematicsCallback, this);
}

SrKinematics::~SrKinematics()
{
}

void SrKinematics::jointstatesCallback( const sensor_msgs::JointStateConstPtr& msg )
{
    mutex.lock();
    //TODO optimize this crappy hack
    for( unsigned int index = 0; index < msg->name.size(); ++index )
    {
        for(unsigned int index_names = 0; index_names < joint_names.size(); ++index_names)
        {
            //if the name is part of the list
            if(boost::find_first(joint_names[index_names], msg->name[index]))
            {
                joints_map[msg->name[index]] = msg->position[index];
                continue;
            }
        }
    }
    joints_map["arm_link"] = 0.0;
    mutex.unlock();
}

void SrKinematics::reverseKinematicsCallback( const tf::tfMessageConstPtr& msg )
{
    tf::StampedTransform transform;
    std::string link_name = msg->transforms[0].child_frame_id;
    try
    {
        //Only compute the reverse kinematics when receiving something from the threedmouse.
        //TODO change this for a more generic way of handling it.
        if( boost::find_first(link_name, "threedmouse") )
        {
            //threedmouse is publishing the transform between the hand support and the threedmouse
            // => we need to get the transform from the arm support (fixed point for the arm) to the threedmouse
            tf_listener.lookupTransform("/sr_arm/position/shadowarm_base", "/threedmouse", ros::Time(0), transform);

            kinematics_msgs::GetPositionIK srv;
            srv.request.ik_request.ik_link_name = "/threedmouse";
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

            srv.request.ik_request.pose_stamped = pose;
            srv.request.timeout = ros::Duration(0.5);

            mutex.lock();
            for( JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it )
            {
                ROS_DEBUG("pos[%s]: %f", it->first.c_str(), it->second);

                srv.request.ik_request.ik_seed_state.joint_state.name.push_back(it->first);
                srv.request.ik_request.ik_seed_state.joint_state.position.push_back(double(it->second));
            }
            mutex.unlock();

            if( rk_client.call(srv) )
            {
                sr_hand::sendupdate msg;
                std::vector<sr_hand::joint> jointVector;

                for( unsigned int i = 0; i < srv.response.solution.joint_state.name.size(); ++i )
                {
                    std::string sensor_name = srv.response.solution.joint_state.name[i];
                    double target = srv.response.solution.joint_state.position[i] * 180.0 / 3.14159;
                    ROS_DEBUG("[%s] = %f", sensor_name.c_str(), target);

                    //fill the sendupdate message
                    sr_hand::joint joint;
                    joint.joint_name = sensor_name;
                    joint.joint_target = target;
                    jointVector.push_back(joint);
                }

                msg.sendupdate_length = jointVector.size();
                msg.sendupdate_list = jointVector;
                //publish the message

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

}
; //end namespace


int main( int argc, char** argv )
{
    ros::init(argc, argv, "sr_kinematics");
    boost::shared_ptr<shadowrobot::SrKinematics> sr_kin(new shadowrobot::SrKinematics::SrKinematics());
    ros::spin();
}

