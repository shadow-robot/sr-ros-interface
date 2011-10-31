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
namespace denso
{
  const unsigned short DensoArm::nb_joints_ = 6; // TODO: I don't know how many joints we have

  DensoArm::DensoArm()
  {}

  DensoArm::DensoArm(string robot_ip, int robot_port, float initial_speed)
  {
    start_bcap(robot_ip, robot_port);
    start_controller();
    start_slave_task();
    take_robot();
    set_speed(initial_speed);
    initialise_position_handles();
    set_power(1);
  }

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

  bool DensoArm::sendupdate( const std::vector<double>& joint_targets )
  {
    sprintf(command_buffer, "J(%f,%f,%f,%f,%f,%f)",joint_targets[0], joint_targets[1], joint_targets[2], joint_targets[3], joint_targets[4], joint_targets[5] );

    BCAP_HRESULT hr = bCap_RobotMove(socket_handle, robot_handle, 2 ,command_buffer, (char * ) "");

    return (hr != BCAP_E_ROBOTISBUSY);
  }

  ////////

  ///////
  // IN CARTESIAN SPACE

  bool DensoArm::send_cartesian_position( const Pose& pose )
  {
    sprintf(command_buffer, "P(%f,%f,%f,%f,%f,%f,1)", pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);

    BCAP_HRESULT hr = bCap_RobotMove(socket_handle, robot_handle, 2 ,command_buffer, (char * ) "");

    return (hr != BCAP_E_ROBOTISBUSY);
  }


  void DensoArm::get_cartesian_position( boost::shared_ptr<Pose> pose )
  {
    //TODO: read the current tip coordinates

    float position[7];

    BCAP_HRESULT hr = bCap_VariableGetValue(socket_handle, cartesian_position_handle, position);			/* Get Value */
    
    if FAILED(hr)
    {
      fprintf (stderr, "Couldn't read position - %i", (int) hr);
      std::exit((int) hr);
    }

    pose->x = position[0];
    pose->y = position[1];
    pose->z = position[2];
    pose->roll = position[3];
    pose->pitch = position[4];
    pose->yaw = position[5];
  }
  ////////

  void DensoArm::start_bcap (string robot_ip, int robot_port)
  {
    BCAP_HRESULT hr = BCAP_S_OK;
    if FAILED(hr = bCap_Open(robot_ip.c_str(), robot_port, &socket_handle))
    {	/* Init socket  */
      fprintf(stderr, "Failed to init sockets - %i\n", hr);
      std::exit((int) hr);
    }

    if FAILED(hr = bCap_ServiceStart(socket_handle)) {    /* Start b-CAP service */
      fprintf(stderr, "Failed to start b-Cap - %i\n", hr);
      std::exit((int) hr);
    }
  }
  
  void DensoArm::start_controller(void)
  {
    BCAP_HRESULT hr = BCAP_S_OK;

    if FAILED(hr = bCap_ControllerConnect(socket_handle, (char*)"", (char*)"", (char*)"", (char*)"", &controller_handle) )
    {
      fprintf(stderr, "Failed to start controller - %i\n", hr);
      std::exit((int) hr);
    }

  }

  void DensoArm::take_robot(void)
  {
    BCAP_HRESULT hr = bCap_ControllerGetRobot(socket_handle, controller_handle, (char*)"", (char*)"", &robot_handle);
    if FAILED(hr)
    {
      fprintf(stderr, "Coudlnt take robot - %i\n", hr);
      exit ((int) hr);
    }
  }

  void DensoArm::set_power(int power_state)
  {
    BCAP_HRESULT hr;
    void * result;

    hr = bCap_RobotExecute(socket_handle, robot_handle, (char*)"MOTOR", VT_I2, 0, &power_state, result);

    if FAILED(hr)
    {
      fprintf(stderr, "Couldn't switch power- %i\n", hr);
      exit ((int) hr);
    }
  }

  void DensoArm::set_speed(float speed)
  {
    float in[3];
    in[0] = speed;
    in[1] = 10;
    in[2] = 10;
    void * result;
    BCAP_HRESULT hr = bCap_RobotExecute(socket_handle, robot_handle, (char*)"ExtSpeed", VT_R4 | VT_ARRAY, 1 , in, result);

    if FAILED(hr)
    {
      printf("Couldn't set speed - %x\n", hr);
      exit ((int) hr);
    }
  }

  void DensoArm::initialise_position_handles (void)
  {
    BCAP_HRESULT hr = bCap_ControllerGetVariable(socket_handle, controller_handle, (char*)"@CURRENT_POSITION", (char*)"", &cartesian_position_handle);	/* Get var handle  */

    if SUCCEEDED(hr)
      BCAP_HRESULT hr = bCap_ControllerGetVariable(socket_handle, controller_handle, (char*)"@CURRENT_ANGLE", (char*)"", &angle_position_handle);	/* Get var handle  */
    
    if FAILED(hr)
    {
      fprintf(stderr, "Couldn't get variable handle - %i\n", hr);
      exit ((int) hr);
    }
  }

  void DensoArm::start_slave_task (void)
  {
    BCAP_HRESULT hr = bCap_ControllerGetTask(socket_handle, controller_handle,(char*) "RobSlave", (char*)"", &slave_task_handle);
    if SUCCEEDED(hr)
      hr = bCap_TaskStop(socket_handle, slave_task_handle, 4, (char*)"");

    if SUCCEEDED(hr)
      hr = bCap_TaskStart(socket_handle, slave_task_handle, 1 ,(char*)"");

    if FAILED(hr)
      std::exit ((int) hr);
  }

}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
