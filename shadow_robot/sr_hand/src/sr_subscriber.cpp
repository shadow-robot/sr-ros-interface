/**
 * @file   sr_subscriber.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 25 15:34:37 2010
 * 
 * @brief  The role of this ROS subscriber is to receive commands
 * messages (sent by a publisher) and pass them to the hand after
 * translating them to the correct format. 
 * 
 * 
 */

#include <iostream>
#include <kdl_parser/kdl_parser.hpp>

#include "sr_hand/sr_subscriber.h"

using namespace std;

namespace
{
vector<string> init_end_effector_names()
{
    vector<string> endeffnames;
    endeffnames.push_back("ffdistal");
    return endeffnames;
}

//TODO do this properly
KDL::JntArray init_joints_min()
{
    KDL::JntArray jmin(19);
    for( unsigned int i = 0; i < 19; ++i )
        jmin(i) = 0.0;
    return jmin;
}
KDL::JntArray init_joints_max()
{
    KDL::JntArray jmax(19);
    for( unsigned int i = 0; i < 19; ++i )
        jmax(i) = KDL::PI / 2.0;

    return jmax;
}
}

namespace shadowrobot
{

const vector<string> SRSubscriber::end_effector_names = init_end_effector_names();
const KDL::JntArray SRSubscriber::joints_min = init_joints_min();
const KDL::JntArray SRSubscriber::joints_max = init_joints_max();

/////////////////////////////////
//    CONSTRUCTOR/DESTRUCTOR   //
/////////////////////////////////

SRSubscriber::SRSubscriber( boost::shared_ptr<SRArticulatedRobot> sr_art_robot ) :
    n_tilde("~"), treeFkSolverPos(sr_kinematic_tree), treeIkSolverVel(sr_kinematic_tree, vector<string> ()), treeSolverPos_NR_JL(sr_kinematic_tree, vector<string> (), KDL::JntArray(),
                                                                                                                                 KDL::JntArray(), treeFkSolverPos, treeIkSolverVel, 100, 1e-6)
{
    sr_articulated_robot = sr_art_robot;

    ///////
    // Initialize the subscribers
    //////
    SRSubscriber::init();
}

SRSubscriber::SRSubscriber( boost::shared_ptr<SRArticulatedRobot> sr_art_robot, KDL::Tree tree ) :
    n_tilde("~"), sr_kinematic_tree(tree), treeFkSolverPos(sr_kinematic_tree), treeIkSolverVel(sr_kinematic_tree, end_effector_names), treeSolverPos_NR_JL(sr_kinematic_tree, end_effector_names,
                                                                                                                                                           joints_min, joints_max, treeFkSolverPos,
                                                                                                                                                           treeIkSolverVel, 100, 1e-6)

{
    sr_articulated_robot = sr_art_robot;

    ///////
    // Initialize the subscribers
    //////
    SRSubscriber::init();

    // subscribe to reverseKinematics topic, set
    // reverseKinematicsCallback function
    std::string prefix;
    std::string searched_param;
    n_tilde.searchParam("shadowhand_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "reverseKinematics";
    reverse_kinematics_sub = node.subscribe(full_topic, 10, &SRSubscriber::reverseKinematicsCallback, this);
}

SRSubscriber::~SRSubscriber()
{
    //    if( shadowhand != NULL )
    //      delete shadowhand;
}

void SRSubscriber::init()
{
    // subscribe to sendupdate topic, set sendupdateCallback function
    std::string prefix;
    std::string searched_param;
    n_tilde.searchParam("shadowhand_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "sendupdate";

    sendupdate_sub = node.subscribe(full_topic, 2, &SRSubscriber::sendupdateCallback, this);

    // subscribe to contrlr topic, set contrlrCallback function
    full_topic = prefix + "contrlr";
    contrlr_sub = node.subscribe(full_topic, 2, &SRSubscriber::contrlrCallback, this);

    // subscribe to config topic, set configCallback function
    full_topic = prefix + "config";
    config_sub = node.subscribe(full_topic, 2, &SRSubscriber::configCallback, this);
}

/////////////////////////////////
//         CALLBACK            //
/////////////////////////////////
void SRSubscriber::sendupdateCallback( const sr_robot_msgs::sendupdateConstPtr& msg )
{
    //loop on all the sendupdate messages received (if > 0)
    int sendupdate_length = msg->sendupdate_length;
    if( sendupdate_length == 0 )
    {
        ROS_WARN("Received empty sendupdate command.");
        return;
    }
    //OK, not empty => loop to process all the sendupdate messages
    for( unsigned short index_msg = 0; index_msg < msg->sendupdate_length; ++index_msg )
    {
        float target = msg->sendupdate_list[index_msg].joint_target;
        string sensor_name = msg->sendupdate_list[index_msg].joint_name;

        ROS_DEBUG("Received sendupdate Command [%s : %f]", sensor_name.c_str(), target);
        sr_articulated_robot->sendupdate(sensor_name, (double)target);
    }

}

void SRSubscriber::contrlrCallback( const sr_robot_msgs::contrlrConstPtr& msg )
{

    vector<string> list_of_parameters = msg->list_of_parameters;

    JointControllerData ctrl_data;

    //parses all the parameters transmitted in the msg
    for( unsigned int index_param = 0; index_param < msg->length_of_list; ++index_param )
    {
        //split the string (around ":")
        vector<string> splitted_string;
        boost::split(splitted_string, msg->list_of_parameters[index_param], boost::is_any_of(":"));

        Parameters param;
        param.name = splitted_string[0];
        param.value = splitted_string[1];

        ctrl_data.data.push_back(param);
    }

    sr_articulated_robot->setContrl(msg->contrlr_name, ctrl_data);
}

void SRSubscriber::configCallback( const sr_robot_msgs::configConstPtr& msg )
{
    ROS_ERROR("Configuration command callback not implemented yet");
}

void SRSubscriber::reverseKinematicsCallback( const sr_robot_msgs::reverseKinematicsConstPtr& msg )
{
    //get cartesian target from the received message
    ROS_ERROR("toto1");
    //KDL::Frames 
    std::map<std::string, KDL::Frame> cartesian_targets;
    cartesian_targets["ffdistal"] = KDL::Frame(KDL::Vector(0.027, -0.050, 0.376));
    //KDL::Frame(KDL::Vector(0.027,0.050,0.376)));

    ROS_ERROR("toto2: %d", sr_kinematic_tree.getNrOfSegments());
    sr_kinematic_tree.getSegment("ffdistal");
    ROS_ERROR("glasp");

    //current positions used as a guess for the inverse kinematics
    KDL::JntArray current_positions(sr_kinematic_tree.getNrOfSegments());
    for( unsigned int i = 0; i < sr_kinematic_tree.getNrOfSegments(); ++i )
    {
        //      ROS_ERROR("segment[%d]: %s",i, shadowhand_kinematic_tree[i]);
        current_positions(i) = 0.0;
    }
    //TODO read data from the hand
    KDL::JntArray computed_positions(sr_kinematic_tree.getNrOfSegments());

    ROS_ERROR("toto3");
    int kinematics_status = -1;
    kinematics_status = treeSolverPos_NR_JL.CartToJnt(current_positions, cartesian_targets, computed_positions);
    ROS_ERROR("toto4");
    if( kinematics_status < 0 )
    {
        ROS_WARN("Could not calculate inverse kinematics");
    }
    else
    {
        for( unsigned int i = 0; i < 19; ++i )
        {
            ROS_ERROR("joint[%d] = %f", i, computed_positions(i));
        }
    }
}

} // end namespace
