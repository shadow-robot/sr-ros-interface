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

namespace shadowrobot
{
const unsigned int SrKinematics::number_of_joints = 4;

SrKinematics::SrKinematics()
{
}

SrKinematics::SrKinematics( KDL::Tree tree ) :
    n_tilde("~")
{
    /* DFS: code to get model from parameter server */

    std::string root_name = "shadowarm_base";
    std::string tip_name = "shadowarm_handsupport_motor";

    std::string urdf_xml, full_urdf_xml;
    n_tilde.param("robot_description", urdf_xml, std::string("robot_description"));
    n_tilde.searchParam(urdf_xml, full_urdf_xml);

    TiXmlDocument xml;
    ROS_DEBUG("Reading xml file from parameter server\n");
    std::string result;
    if( n_tilde.getParam(full_urdf_xml, result) )
        xml.Parse(result.c_str());
    else
    {
        ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
        return;
    }
    std::string xml_string = result;
    TiXmlElement *root_element = xml.RootElement();
    TiXmlElement *root = xml.FirstChildElement("robot");
    if( !root || !root_element )
    {
        ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
        exit(1);
    }

    robot_model.initXml(root);

    if( !tree.getNrOfSegments() )
    {
        ROS_ERROR("empty tree. sad.");
        return;
    }

    if( !tree.getChain(root_name, tip_name, chain) )
    {
        ROS_ERROR("couldn't pull arm chain from robot model");
        return;
    }
    ROS_INFO("parsed tree successfully");

    unsigned int num_joints = 0;

    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    while( link && link->name != root_name )
    {
        boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
        ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
        if( !joint )
        {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return;
        }
        if( joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED )
        {
            num_joints++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }

    KDL::SegmentMap::const_iterator root_seg = tree.getRootSegment();
    std::string tree_root_name = root_seg->first;
    ROS_INFO("root: %s", tree_root_name.c_str());
    fk_solver_chain =  boost::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(chain));
    ik_solver_vel = boost::shared_ptr<KDL::ChainIkSolverVel_pinv>(new KDL::ChainIkSolverVel_pinv(chain));
    sensor_msgs::JointState g_js, g_actual_js;

    q_min.resize(num_joints);
    q_max.resize(num_joints);
    g_actual_js.name.resize(num_joints);
    g_actual_js.position.resize(num_joints);
    g_js.name.resize(num_joints);
    g_js.position.resize(num_joints);

    link = robot_model.getLink(tip_name);

    unsigned int i = 0;
    while( link && i < num_joints )
    {
        boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
        ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );
        if( !joint )
        {
            ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
            return;
        }
        if( joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED )
        {
            if( joint->type != urdf::Joint::CONTINUOUS )
            {
                g_js.name[num_joints - i - 1] = joint->name;
                q_min.data[num_joints - i - 1] = joint->limits->lower;
                q_max.data[num_joints - i - 1] = joint->limits->upper;

                ROS_INFO("joint %s: min = %f, max = %f", joint->name.c_str(), joint->limits->lower, joint->limits->upper);
            }
            else
            {
                g_js.name[num_joints - i - 1] = joint->name;
                q_min.data[num_joints - i - 1] = -M_PI;
                q_max.data[num_joints - i - 1] = M_PI;
            }
            i++;
        }
        link = robot_model.getLink(link->getParent()->name);
    }

    g_ik_solver = boost::shared_ptr<KDL::ChainIkSolverPos_NR_JL>(new KDL::ChainIkSolverPos_NR_JL(chain, q_min, q_max, *fk_solver_chain, *ik_solver_vel, 1000, 1e-2));
    //g_ik_solver = boost::shared_ptr<KDL::ChainIkSolverPos_NR>(new KDL::ChainIkSolverPos_NR(chain, fk_solver_chain, ik_solver_vel, 1000, 1e-1));
}

SrKinematics::~SrKinematics()
{
}

int SrKinematics::computeReverseKinematics( tf::Transform transform, std::vector<double> &joints )
{
    if( !computing_mutex.try_lock() )
        return -1;

    if( joints.size() != number_of_joints )
    {
        ROS_ERROR("Received an initial pose containing %d joints instead of %d.", joints.size(), number_of_joints);

        computing_mutex.unlock();
        return -1;
    }

    KDL::JntArray q_init(number_of_joints), q(number_of_joints);
    for( unsigned int i = 0; i < number_of_joints; ++i )
    {
        q_init.data[i] = joints[i];
    }

    KDL::Frame destination_frame;
    tf::TransformTFToKDL(transform, destination_frame);
    ROS_ERROR("starting");
    if( g_ik_solver->CartToJnt(q_init, destination_frame, q) < 0 )
    {
        ROS_ERROR("ik solver fail");

        computing_mutex.unlock();
        return -1;
    }
    ROS_ERROR("done");

    std::stringstream ss;
    ss << "Joint angles: [";

    bool not_zeros = false;

    for( unsigned int i = 0; i < number_of_joints - 1; ++i )
    {
        if( q.data[i] != 0.0 )
            not_zeros = true;

        ss << q.data[i] << ", ";
        //	joints[i] = q.data[i];
    }
    ss << q.data[number_of_joints - 1] << "]";

    ROS_ERROR("%s", (ss.str()).c_str());

    computing_mutex.unlock();
    return 0;
}

}
; //end namespace
