/**
 * @file   sr_kinematics.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Jul 15 11:54:22 2010
 * 
 * @brief  
 * 
 * 
 */

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <sstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include "robot_state_publisher/treefksolverposfull_recursive.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <sensor_msgs/JointState.h>

#include "sr_hand/sr_kinematics.h"

namespace shadowhand
{
  const unsigned int SrKinematics::number_of_joints = 4;

  SrKinematics::SrKinematics()
  {
  }

  SrKinematics::SrKinematics(KDL::Tree tree)
  {
    /*kinematic_tree = tree;

      if (!tree.getNrOfSegments())
      {
      ROS_ERROR("The given KDL tree is empty.");
      return;
      }

      ROS_INFO("Number of segments in the KDL tree: %d", tree.getNrOfSegments());

      KDL::Chain chain;
      if (!tree.getChain("shadowarm_base", "shadowarm_handsupport", chain))
      {
      ROS_ERROR("couldn't pull arm chain from robot model");
      return;
      }

      ROS_INFO("parsed tree successfully");
   
      KDL::ChainFkSolverPos_recursive fk_solver_chain(chain);
      KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
    
      KDL::JntArray q_min(number_of_joints), q_max(number_of_joints);
      q_min.data[0] = toRad(-45.0);
      q_max.data[0] = toRad(90.0);

      q_min.data[1] = toRad(0.0);
      q_max.data[1] = toRad(90.0);

      q_min.data[2] = toRad(0.0);
      q_max.data[2] = toRad(120.0);

      q_min.data[3] = toRad(-90.0);
      q_max.data[3] = toRad(90.0);
    */

    /* DFS: code to get model from parameter server */
    urdf::Model robot_model;
    std::string robot_desc;
    KDL::Chain chain;
    ros::NodeHandle n;
    ros::NodeHandle n_tilde("~");

    std::string root_name = "shadowarm_base";
    std::string tip_name = "shadowarm_handsupport";
    
    std::string urdf_xml,full_urdf_xml;
    n_tilde.param("robot_description",urdf_xml,std::string("robot_description"));
    n_tilde.searchParam(urdf_xml,full_urdf_xml);


    TiXmlDocument xml;
    ROS_DEBUG("Reading xml file from parameter server\n");
    std::string result;
    if (n_tilde.getParam(full_urdf_xml, result))
      xml.Parse(result.c_str());
    else
      {
	ROS_FATAL("Could not load the xml from parameter server: %s\n", urdf_xml.c_str());
	return ;
      }
    std::string xml_string = result;
    TiXmlElement *root_element = xml.RootElement();
    TiXmlElement *root = xml.FirstChildElement("robot");
    if (!root || !root_element)
      {
	ROS_FATAL("Could not parse the xml from %s\n", urdf_xml.c_str());
	exit(1);
      }

    robot_model.initXml(root);

    if (!tree.getNrOfSegments())
      {
	ROS_ERROR("empty tree. sad.");
	return ;
      }

    if (!tree.getChain(root_name, tip_name, chain))
      {
	ROS_ERROR("couldn't pull arm chain from robot model");
	return;
      }

    ROS_INFO("parsed tree successfully");

    unsigned int num_joints = 0;
    KDL::JntArray q_min, q_max;

    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
    while(link && link->name != root_name)
      {
	boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
	ROS_INFO( "adding joint: [%s]", joint->name.c_str() );
	if(!joint)
	  {
	    ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
	    return ;
	  }
	if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
	  {
	    num_joints++;
	  }
	link = robot_model.getLink(link->getParent()->name);
      }


    KDL::SegmentMap::const_iterator root_seg = tree.getRootSegment();
    std::string tree_root_name = root_seg->first;
    ROS_INFO("root: %s", tree_root_name.c_str());
    KDL::ChainFkSolverPos_recursive fk_solver_chain(chain);
    KDL::ChainIkSolverVel_pinv ik_solver_vel(chain);
    sensor_msgs::JointState g_js, g_actual_js;

    q_min.resize(num_joints);
    q_max.resize(num_joints);
    g_actual_js.name.resize(num_joints);
    g_actual_js.position.resize(num_joints);
    g_js.name.resize(num_joints);
    g_js.position.resize(num_joints);


    link = robot_model.getLink(tip_name);

    unsigned int i = 0;
    while(link && i < num_joints)
      {
	boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
	ROS_INFO( "getting bounds for joint: [%s]", joint->name.c_str() );
	if(!joint)
	  {
	    ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
	    return ;
	  }
	if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
	  {
	    if( joint->type != urdf::Joint::CONTINUOUS )
	      {
		g_js.name[num_joints-i-1] = joint->name;
		q_min.data[num_joints-i-1] = joint->limits->lower;
		q_max.data[num_joints-i-1] = joint->limits->upper;
	      }
	    else
	      {
		g_js.name[num_joints-i-1] = joint->name;
		q_min.data[num_joints-i-1] = -M_PI;
		q_max.data[num_joints-i-1] = M_PI;
	      }
	    i++;
	  }
	link = robot_model.getLink(link->getParent()->name);
      }

    g_ik_solver = boost::shared_ptr<KDL::ChainIkSolverPos_NR_JL>(new KDL::ChainIkSolverPos_NR_JL(chain, q_min, q_max, fk_solver_chain, ik_solver_vel, 100, 0.5));
  }

  SrKinematics::~SrKinematics()
  {
  }

  int SrKinematics::computeReverseKinematics(tf::Transform transform, std::vector<double> &joints)
  {
    if( joints.size() != number_of_joints)
      {
	ROS_ERROR("Received an initial pose containing %d joints instead of %d.", joints.size(), number_of_joints);
	return -1;
      }

    KDL::JntArray q_init(number_of_joints), q(number_of_joints);
    for (unsigned int i = 0; i < number_of_joints; i++)
      {
	q_init.data[i] = joints[i];
      }

    KDL::Frame destination_frame;
    tf::TransformTFToKDL(transform, destination_frame);
    if (g_ik_solver->CartToJnt(q_init, destination_frame, q) < 0)
      {
	ROS_WARN("ik solver fail");
	return -1;
      }

    for (unsigned int i = 0; i < number_of_joints; i++)
      {
	ROS_ERROR("joint_angle[%d], %f", i, q.data[i]);
    
	//	joints[i] = q.data[i];
      }

    ros::spinOnce();
    return 0;
  }


}; //end namespace
