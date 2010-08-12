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
#include <tf_conversions/tf_kdl.h>



namespace shadowhand
{
  class SrKinematics
  {
  public:
    SrKinematics();
    SrKinematics(KDL::Tree tree);
    ~SrKinematics();

    int computeReverseKinematics(tf::Transform transform, std::vector<double> &joints);

  private:
    KDL::Tree kinematic_tree;

    boost::shared_ptr<KDL::ChainIkSolverPos_NR_JL> g_ik_solver;

    static const unsigned int number_of_joints;

    /**
     * Convert an angle in degree to an angle in radians.
     * @param deg the angle in degrees
     * @return the value in rads.
     */
    inline double toRad(double deg)
    {
      return deg * 3.14159265 / 180.0;
    }
  }; // end class SrKinematics

}; //end namespace


#endif

