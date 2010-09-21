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

//kdl
#include <kdl/tree.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <tf_conversions/tf_kdl.h>

namespace shadowrobot
{
class SrKinematics
{
public:
    SrKinematics();
    SrKinematics( KDL::Tree tree );
    ~SrKinematics();

    int computeReverseKinematics( tf::Transform transform, std::vector<double> &joints );

private:
    ros::NodeHandle n, n_tilde;

    urdf::Model robot_model;
    std::string robot_desc;
    KDL::Chain chain;

    KDL::Tree kinematic_tree;

    //kinematics solver
    KDL::JntArray q_min, q_max;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_chain;
    boost::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel;
    boost::shared_ptr<KDL::ChainIkSolverPos_NR_JL> g_ik_solver;
    //boost::shared_ptr<KDL::ChainIkSolverPos_NR> g_ik_solver;

    static const unsigned int number_of_joints;

    /**
     * Convert an angle in degree to an angle in radians.
     * @param deg the angle in degrees
     * @return the value in rads.
     */
    inline double toRad( double deg )
    {
        return deg * 3.14159265 / 180.0;
    }

    boost::mutex computing_mutex;
}; // end class SrKinematics

}
; //end namespace


#endif

