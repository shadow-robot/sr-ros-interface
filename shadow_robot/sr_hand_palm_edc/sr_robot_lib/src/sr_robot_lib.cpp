/**
 * @file   sr_robot_lib.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Jun 22 10:06:14 2011
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
*
 * @brief This is a generic robot library for Shadow Robot's Hardware.
 *
 *
 */

#include "sr_robot_lib/sr_hand_lib.hpp"
#include <string>
#include <boost/foreach.hpp>

#include <sr_utilities/sr_math_utils.hpp>

namespace shadow_robot
{
  void update(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data)
  {
    boost::shared_ptr<shadow_robot::JointCalibration> calibration_tmp;

    //read the PIC idle time
    main_pic_idle_time = status_data->idle_time_us;
    if( status_data->idle_time_us < main_pic_idle_time_min )
      main_pic_idle_time_min = status_data->idle_time_us;

    boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp = joints_vector.begin();
    for(;joint_tmp != joints_vector.end(); ++joint_tmp)
    {
      pr2_hardware_interface::Actuator* actuator = (joint_tmp->motor->actuator);
      pr2_hardware_interface::ActuatorState* state(&actuator->state_);

      int motor_index_full = joint_tmp->motor->motor_id;
      state->is_enabled_ = 1;
      state->device_id_ = motor_index_full;

      /////////////
      // Get the joint positions and compute the calibrate
      // values

      if(joint_tmp->joint_to_sensor.calibrate_after_combining_sensors)
      {
        //first we combine the different sensors and then we
        // calibrate the value we obtained. This is used for
        // some compound sensors ( THJ5 = cal(THJ5A + THJ5B))
        double raw_position = 0.0;
        //when combining the values, we use the coefficient imported
        // from the sensor_to_joint.yaml file (in sr_edc_launch/config)
        BOOST_FOREACH(shadow_joints::PartialJointToSensor joint_to_sensor, joint_tmp->joint_to_sensor.joint_to_sensor_vector)
          raw_position += static_cast<double>(status_data->sensors[joint_to_sensor.sensor_id])*joint_to_sensor.coeff;

        //That's not an encoder position, just the raw value
        state->encoder_count_ = static_cast<int>(raw_position);

        //and now we calibrate
        calibration_tmp = calibration_map.find(joint_tmp->joint_name);
        state->position_ = calibration_tmp->compute( static_cast<double>(raw_position) );
      }
      else
      {
        //we calibrate the different sensors first and we combine the calibrated
        //values. This is used in the joint 0s for example ( J0 = cal(J1)+cal(J2) )
        double calibrated_position = 0.0;
        shadow_joints::PartialJointToSensor joint_to_sensor;
        std::string sensor_name;
        for(unsigned int index_joint_to_sensor=0;
            index_joint_to_sensor < joint_tmp->joint_to_sensor.joint_to_sensor_vector.size();
            ++index_joint_to_sensor)
        {
          joint_to_sensor = joint_tmp->joint_to_sensor.joint_to_sensor_vector[index_joint_to_sensor];
          sensor_name = joint_tmp->joint_to_sensor.sensor_names[index_joint_to_sensor];
          //get the raw position
          double raw_pos = static_cast<double>(status_data->sensors[joint_to_sensor.sensor_id]);
          //calibrate and then combine
          calibration_tmp = calibration_map.find(sensor_name);
          calibrated_position += calibration_tmp->compute(raw_pos) * joint_to_sensor.coeff;
        }
        state->position_ = calibrated_position;
      }

      //
      ////////////

      //if no motor is associated to this joint, then continue
      if( (motor_index_full == -1) )
        continue;

      //get the remaining information.
      bool read_motor_info = false;
      int index_motor_in_msg = 0;

      if(status_data->which_motors == 0)
      {
        //We sampled the even motor numbers
        if( motor_index_full%2 == 0)
          read_motor_info = true;
      }
      else
      {
        //we sampled the odd motor numbers
        if( motor_index_full%2 == 1)
          read_motor_info = true;
      }

      //the position of the motor in the message
      // is different from the motor index:
      // the motor indexes range from 0 to 19
      // while the message contains information
      // for only 10 motors.
      index_motor_in_msg = motor_index_full/2;

      //setting the position of the motor in the message,
      // we'll print that in the diagnostics.
      joint_tmp->motor->msg_motor_id = index_motor_in_msg;

      //ok now we read the info and add it to the actuator state
      if(read_motor_info)
      {
        //ROS_DEBUG_STREAM("Reading motor: "<<joint_tmp->joint_name << " ("<< motor_index_full << " / "<< index_motor_in_msg<<") => mask = "<<status_data->which_motor_data_arrived);

        //check the masks to see if the CAN messages arrived to the motors
        //the flag should be set to 1 for each motor
        joint_tmp->motor->motor_ok = sr_math_utils::is_bit_mask_index_true(status_data->which_motor_data_arrived, motor_index_full);

        //check the masks to see if a bad CAN message arrived
        //the flag should be 0
        joint_tmp->motor->bad_data = sr_math_utils::is_bit_mask_index_true(status_data->which_motor_data_had_errors, index_motor_in_msg);

        if(joint_tmp->motor->motor_ok && !(joint_tmp->motor->bad_data) )
        {
          //TODO: Hugo: how can I get the correct values from the motor???

          //we received the data and it was correct
          switch(status_data->motor_data_type)
          {
          case MOTOR_DATA_SGL:
            joint_tmp->motor->strain_gauge_left =  status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_SGR:
            joint_tmp->motor->strain_gauge_right =  status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_PWM:
            state->last_executed_effort_ =  (double)status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_FLAGS:
            joint_tmp->motor->flags = status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_CURRENT:
            //we're receiving the current in milli amps
            state->last_measured_current_ = ((double)status_data->motor_data_packet[index_motor_in_msg].misc)/1000.0;
            break;
          case MOTOR_DATA_VOLTAGE:
            state->motor_voltage_ = ((double)status_data->motor_data_packet[index_motor_in_msg].misc ) / 256.0;
            break;
          case MOTOR_DATA_TEMPERATURE:
            joint_tmp->motor->temperature = ((double)status_data->motor_data_packet[index_motor_in_msg].misc) / 256.0;
            break;
          case MOTOR_DATA_CAN_NUM_RECEIVED:
            joint_tmp->motor->can_msgs_received = status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_CAN_NUM_TRANSMITTED:
            joint_tmp->motor->can_msgs_transmitted = status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_SVN_REVISION:
            joint_tmp->motor->firmware_svn_revision = status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_TESTS:
            joint_tmp->motor->tests = status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_F_P:
            joint_tmp->motor->force_control_p = status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_I_D:
            joint_tmp->motor->force_control_i = status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          case MOTOR_DATA_IMAX_DEADBAND_SIGN:
            joint_tmp->motor->force_control_imax = status_data->motor_data_packet[index_motor_in_msg].misc;
            break;
          default:
            break;
          }

          state->last_measured_effort_ = (double)status_data->motor_data_packet[index_motor_in_msg].torque;
        }
      }

    } //end BOOST_FOREACH joint names

  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
