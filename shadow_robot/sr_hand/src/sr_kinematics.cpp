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

#include "sr_hand/sr_kinematics.h"

namespace shadowhand
{
  const unsigned int SrKinematics::number_of_joints = 4;

  SrKinematics::SrKinematics()
  {
  }

  SrKinematics::SrKinematics(KDL::Tree tree)
  {
    kinematic_tree = tree;

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


    g_ik_solver = boost::shared_ptr<KDL::ChainIkSolverPos_NR_JL>(new KDL::ChainIkSolverPos_NR_JL(chain, q_min, q_max,
												 fk_solver_chain, ik_solver_vel, 
												 100, 1e-6));
  }

  SrKinematics::~SrKinematics()
  {
  }

  int SrKinematics::computeReverseKinematics(tf::Transform t, std::vector<double> &pose)
  {
    if( pose.size() != number_of_joints)
      {
	ROS_ERROR("Received an initial pose containing %d joints instead of %d.", pose.size(), number_of_joints);
	return -1;
      }

    KDL::JntArray q_init(number_of_joints), q(number_of_joints);
    for (unsigned int i = 0; i < number_of_joints; i++)
      q_init.data[i] = pose[i];

    // populate F_dest from tf::Transform parameter
    KDL::Frame F_dest;
    tf::TransformTFToKDL(t, F_dest);
    if (g_ik_solver->CartToJnt(q_init, F_dest, q) < 0)
      {
	ROS_ERROR("ik solver fail");
	return false;
      }
    for (unsigned int i = 0; i < number_of_joints; i++)
      pose[i] = q.data[i];

    return 0;
  }
}; //end namespace
