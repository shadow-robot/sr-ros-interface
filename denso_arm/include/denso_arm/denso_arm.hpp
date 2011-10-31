/**
 * @file   denso_arm.hpp
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

#ifndef _DENSO_ARM_HPP_
#define _DENSO_ARM_HPP_

#include "denso_arm/denso_joints.hpp"
#include <boost/smart_ptr.hpp>
#include "b-Cap/b-Cap.hpp"
#include <cstdlib>

namespace denso
{
  struct Pose
  {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
  };

  class DensoArm
  {
  public:
    DensoArm();
    DensoArm(const char * pStrIP, int iPort, float fInitialSpeed);//Dan: You can get whatever you want in this constructor
    virtual ~DensoArm();

    void update_state(boost::shared_ptr<DensoJointsVector> denso_joints);
    void sendupdate( int index_joint, double target );

    /**
     * Send a cartesian demand to the Denso arm.
     *
     * @param pose the target in the robot frame
     *
     * @return true if target reached
     */
    bool send_cartesian_position( const Pose& pose );

    void get_cartesian_position( boost::shared_ptr<Pose> pose);

    unsigned short get_nb_joints()
    {
      return nb_joints_;
    };

  private:
    int ihSocket;
    u_long lhController;
    u_long lhRobot;
    u_long lhSlaveTask;
    u_long lhPosition;


    void StartBCAP (const char * pStrIP, int iPort);
    void StartController(void);
    void TakeRobot(void);
    void SetPower(int iPower);
    void SetSpeed(float fSpeed);
    void InitialisePositionHandle (void);
    void StartRobSlave(void);



  protected:
    static const unsigned short nb_joints_;
  };
}


  /* For the emacs weenies in the crowd.
     Local Variables:
     c-basic-offset: 2
     End:
  */

#endif
