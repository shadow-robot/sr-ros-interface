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
#include "b-Cap/b-Cap.hpp"
#include <sstream>

#include <sr_utilities/sr_math_utils.hpp>

namespace denso
{
  const unsigned short DensoArm::nb_joints_ = 6;

  DensoArm::DensoArm()
  {
    init("10.2.2.222", 5007, 5);
  }

  DensoArm::DensoArm(std::string robot_ip, int robot_port, float initial_speed)
  {
    init(robot_ip, robot_port, initial_speed);
  }

  void DensoArm::init(std::string robot_ip, int robot_port, float initial_speed)
  {
    start_bcap(robot_ip, robot_port);
    usleep(1000);
    start_controller();
    usleep(1000);
    start_slave_task();
    usleep(1000);
    take_robot();
    usleep(1000);
    set_tooltip(0);
    usleep(1000);
    set_speed(initial_speed);
    usleep(1000);
    initialise_position_handles();
    usleep(1000);
    set_power(1);
  }

  DensoArm::~DensoArm()
  {
    set_power (0);
    release_position_handles();

    bCap_RobotRelease(socket_handle, robot_handle);

    stop_slave_task();
    bCap_ControllerDisconnect(socket_handle,controller_handle);
    bCap_Close(socket_handle);
  }


  ///////
  // IN JOINT SPACE

  void DensoArm::update_state(boost::shared_ptr<DensoJointsVector> denso_joints)
  {
    static float position[nb_joints_];
    BCAP_HRESULT hr = bCap_VariableGetValue(socket_handle, joint_position_handle, position);			/* Get Value */

    if( FAILED(hr) )
      ROS_ERROR("Couldn't read joint positions - %d", (int) hr);

    for (unsigned short index_joint = 0; index_joint < nb_joints_; ++index_joint)
    {
      denso_joints->at(index_joint).position = sr_math_utils::to_rad( position[index_joint] );

      //We don't seem to have access to those
      denso_joints->at(index_joint).effort = 0.0;
      denso_joints->at(index_joint).velocity = 0.0;
    }
  }

  bool DensoArm::sendupdate( const std::vector<double>& joint_targets )
  {
    std::stringstream command;
    command << "J(" << joint_targets[0];
    for (unsigned short index_joint = 1; index_joint < nb_joints_; index_joint++)
    {
      command << "," << joint_targets[index_joint];
    }
    command << ")";

    BCAP_HRESULT hr = bCap_RobotMove(socket_handle, robot_handle, 2 ,(char *) command.str().c_str(), (char * ) "");

    if (FAILED(hr) && (hr != BCAP_E_ROBOTISBUSY) )
      ROS_ERROR("Couldn't move robot - %d", (int) hr);

    return (hr != BCAP_E_ROBOTISBUSY);  // TODO Still returns 1 if there is an error...
  }

  ////////

  ///////
  // IN CARTESIAN SPACE

  bool DensoArm::set_tooltip( int tool_number )
  {
      std::stringstream command;
      command << "Tool="<<tool_number;

      BCAP_HRESULT hr = bCap_RobotChange(socket_handle, robot_handle, (char *) command.str().c_str());

      if( FAILED(hr) )
        printf("Couldn't set tool number - %x\n", hr);
    return true;
  }

  bool DensoArm::send_cartesian_position( const Pose& pose )
  {
    std::stringstream command;
    command << "P(" <<  pose.x << "," <<  pose.y << "," << pose.z << "," << pose.roll << "," << pose.pitch << "," << pose.yaw << ")";
    ROS_DEBUG_STREAM(" moving to: " << command.str() );

    BCAP_HRESULT hr = bCap_RobotMove(socket_handle, robot_handle, 2 ,(char *) command.str().c_str(), (char * ) "");

    if (FAILED(hr) && (hr != BCAP_E_ROBOTISBUSY) )
      ROS_ERROR("Couldn't move robot - %x", (int) hr);

    return ( SUCCEEDED(hr) );
  }


  void DensoArm::get_cartesian_position( boost::shared_ptr<Pose> pose )
  {
    static float position[7];

    BCAP_HRESULT hr = bCap_VariableGetValue(socket_handle, cartesian_position_handle, position);			/* Get Value */

    if( FAILED(hr) )
      ROS_ERROR("Couldn't read position - %i", (int) hr);

    pose->x = position[0];
    pose->y = position[1];
    pose->z = position[2];
    pose->roll = position[3];
    pose->pitch = position[4];
    pose->yaw = position[5];
  }
  ////////

  void DensoArm::start_bcap (std::string robot_ip, int robot_port)
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    if( FAILED(hr = bCap_Open(robot_ip.c_str(), robot_port, &socket_handle)) )
      ROS_FATAL("Failed to init sockets - %i\n", hr);

    if( FAILED(hr = bCap_ServiceStart(socket_handle)) )
      ROS_FATAL("Failed to start b-Cap - %i\n", hr);
  }

  void DensoArm::start_controller(void)
  {
    BCAP_HRESULT hr = BCAP_S_OK;

    if( FAILED(hr = bCap_ControllerConnect(socket_handle, (char*)"", (char*)"", (char*)"", (char*)"", &controller_handle) ) )
      ROS_FATAL("Failed to start controller - %i\n", hr);
  }

  void DensoArm::take_robot(void)
  {
    BCAP_HRESULT hr = bCap_ControllerGetRobot(socket_handle, controller_handle, (char*)"", (char*)"", &robot_handle);

    if( FAILED(hr) )
      ROS_FATAL("Coudlnt take robot - %i\n", hr);
  }

  void DensoArm::set_power(int power_state)
  {
    BCAP_HRESULT hr;
    void * result = 0;

    hr = bCap_RobotExecute(socket_handle, robot_handle, (char*)"MOTOR", VT_I2, 0, &power_state, result);

    if( FAILED(hr) )
      ROS_ERROR("Couldn't switch power- %i\n", hr);
  }

  void DensoArm::set_speed(float speed)
  {
    float in[3];
    in[0] = speed;
    in[1] = 10;
    in[2] = 10;
    void * result = 0;
    BCAP_HRESULT hr = bCap_RobotExecute(socket_handle, robot_handle, (char*)"ExtSpeed", VT_R4 | VT_ARRAY, 1 , in, result);

    if( FAILED(hr) )
      ROS_ERROR("Couldn't set speed - %x\n", hr);
  }
  void DensoArm::release_position_handles()
  {
    bCap_VariableRelease(socket_handle, cartesian_position_handle);
    bCap_VariableRelease(socket_handle, joint_position_handle);
  }

  void DensoArm::initialise_position_handles (void)
  {
    BCAP_HRESULT hr = bCap_RobotGetVariable(socket_handle, robot_handle, (char*)"@CURRENT_POSITION", (char*)"", &cartesian_position_handle);	/* Get var handle  */

    if( FAILED(hr) )
      ROS_ERROR("Couldn't get cartesian variable handle - %i\n", hr);

    hr = bCap_RobotGetVariable(socket_handle, robot_handle, (char*)"@CURRENT_ANGLE", (char*)"", &joint_position_handle);	/* Get var handle  */

    if( FAILED(hr) )
      ROS_ERROR("Couldn't get angle variable handle - %i\n", hr);

  }

  void DensoArm::stop_slave_task (void)
  {
    bCap_TaskStop(socket_handle, slave_task_handle, 4, (char*)"");
    bCap_TaskRelease(socket_handle, slave_task_handle);
  }


  void DensoArm::start_slave_task (void)
  {
    BCAP_HRESULT hr = bCap_ControllerGetTask(socket_handle, controller_handle,(char*) "RobSlave", (char*)"", &slave_task_handle);
    if( SUCCEEDED(hr) )
      hr = bCap_TaskStop(socket_handle, slave_task_handle, 4, (char*)"");

    if( SUCCEEDED(hr) )
      hr = bCap_TaskStart(socket_handle, slave_task_handle, 1 ,(char*)"");

    if( FAILED(hr) )
      ROS_FATAL("Couldn't start RobSlave - %d", (int) hr);
  }

}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
