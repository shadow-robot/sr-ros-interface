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
#include <string>

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
    DensoArm(std::string robot_ip, int robot_port, float intial_speed);

    virtual ~DensoArm();

    void update_state(boost::shared_ptr<DensoJointsVector> denso_joints);

    bool sendupdate( const std::vector<double>& joint_targets );

    /**
     * Send a cartesian demand to the Denso arm.
     *
     * @param pose the target in the robot frame
     *
     * @return true if target reached
     */
    bool send_cartesian_position( const Pose& pose );

    void get_cartesian_position( boost::shared_ptr<Pose> pose);

    /**
     * Sets the pose of the tooltip for the IK.
     *
     * @param tool_number the number of the tool in the denso arm.
     *
     * @return true if success
     */
    bool set_tooltip( int tool_number );

    void set_speed(float speed);

    unsigned short get_nb_joints()
    {
      return nb_joints_;
    };

  protected:
    static const unsigned short nb_joints_;

    int socket_handle;
    u_long controller_handle;
    u_long robot_handle;
    u_long slave_task_handle;
    u_long cartesian_position_handle;
    u_long joint_position_handle;

    void init(std::string robot_ip, int robot_port, float initial_speed);

    void start_bcap (std::string robot_ip, int robot_port);
    void start_controller(void);
    void take_robot(void);
    void set_power(int power_state);
    void initialise_position_handles (void);
    void start_slave_task(void);
    void stop_slave_task(void);
    void release_position_handles(void);
  };
}


  /* For the emacs weenies in the crowd.
     Local Variables:
     c-basic-offset: 2
     End:
  */

#endif
