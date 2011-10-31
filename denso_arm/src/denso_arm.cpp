/**
 * @file   denso_arm.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Dan Greenwald <dg@shadowrobot.com>
 * @date   Mon Oct 31 09:26:15 2011
 *
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief A standard interface to the DENSO arm.
 *
 *
 */

#include "denso_arm/denso_arm_node.hpp"

namespace denso
{
  const unsigned short DensoArm::nb_joints_ = 7; // TODO: I don't know how many joints we have

  DensoArm::DensoArm()//Dan: You can get whatever you want in this constructor
  {}

  DensoArm::~DensoArm()
  {}


  ///////
  // IN JOINT SPACE

  void DensoArm::update_state(boost::shared_ptr<DensoJointsVector> denso_joints)
  {
    for (unsigned short index_joint = 0; index_joint < nb_joints_; ++index_joint)
    {
      //TODO: update the vector: read the real values
      denso_joints->at(index_joint).position = 0.0;
      denso_joints->at(index_joint).effort = 0.0;
      //do you have access to this?
      denso_joints->at(index_joint).velocity = 0.0;
    }
  }

  void DensoArm::sendupdate( int index_joint, double target )
  {
    //TODO: send the target to the correct joint.
  }

  ////////

  ///////
  // IN CARTESIAN SPACE

  bool DensoArm::send_cartesian_position( const Pose& pose )
  {
    //TODO: send those coordinates to the arm
    // pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw

    return true;
  }


  void DensoArm::get_cartesian_position( boost::shared_ptr<Pose> pose )
  {
    //TODO: read the current tip coordinates
    pose->x = 0.0;
    pose->y = 0.0;
    pose->z = 0.0;
    pose->roll = 0.0;
    pose->yaw = 0.0;
    pose->pitch = 0.0;
  }
  ////////

}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
