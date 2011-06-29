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

#include <ros/ros.h>

namespace shadow_robot
{
  SrRobotLib::SrRobotLib(pr2_hardware_interface::HardwareInterface *hw)
    : main_pic_idle_time(0), main_pic_idle_time_min(1000), config_index(MOTOR_CONFIG_FIRST_VALUE), nh_tilde("~")
  {
    debug_publishers.push_back(node_handle.advertise<std_msgs::Int16>("sr_debug_1",100));
    debug_publishers.push_back(node_handle.advertise<std_msgs::Int16>("sr_debug_2",100));
  }


  void SrRobotLib::update(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data)
  {
    this->status_data = status_data;

    //read the PIC idle time
    main_pic_idle_time = status_data->idle_time_us;
    if( status_data->idle_time_us < main_pic_idle_time_min )
      main_pic_idle_time_min = status_data->idle_time_us;

    boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp = joints_vector.begin();
    for(;joint_tmp != joints_vector.end(); ++joint_tmp)
    {
      actuator = (joint_tmp->motor->actuator);

      motor_index_full = joint_tmp->motor->motor_id;
      actuator->state_.is_enabled_ = 1;
      actuator->state_.device_id_ = motor_index_full;

      //calibrate the joint and update the position.
      calibrate_joint(joint_tmp);

      //if no motor is associated to this joint, then continue
      if( (motor_index_full == -1) )
        continue;

      //get the remaining information.
      bool read_motor_info = false;

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
        read_additional_data(joint_tmp);

    } //end BOOST_FOREACH joint names
  } //end update()

  void SrRobotLib::build_motor_command(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command)
  {
    motor_updater_->build_update_motor_command(command);

    ///////
    // Now we chose the command to send to the motor
    // by default we send a torque demand (we're running
    // the force control on the motors), but if we have a waiting
    // configuration, then we send the configuration.

    if( reconfig_queue.empty() )
    {
      //no config to send
      //command->to_motor_data_type   = MOTOR_DEMAND_TORQUE;
      //TODO: change back to torque
      command->to_motor_data_type   = MOTOR_DEMAND_PWM;

      //loop on either even or odd motors
      int motor_index = 0;
      for(unsigned int i = 0; i < NUM_MOTORS; ++i)
        command->motor_data[i] = joints_vector[motor_index].motor->actuator->command_.effort_;
    } //endif reconfig_queue.empty()
    else
    {
      //we have a waiting config:
      // we need to send all the config, finishing by the
      // CRC. We'll remove the config from the queue only
      // when the whole config has been sent

      // the motor data type correspond to the index
      // in the config array.
      command->to_motor_data_type   = static_cast<TO_MOTOR_DATA_TYPE>(config_index);

      //convert the motor index to the index of the motor in the message
      int motor_index = reconfig_queue.front().first;

      //set the data we want to send to the given motor
      command->motor_data[motor_index] = reconfig_queue.front().second[config_index].word;

      //We're now sending the CRC. We need to send the correct CRC to
      // the motor we updated, and CRC=0 to all the other motors in its
      // group (odd/even) to tell them to ignore the new
      // configuration.
      // Once the config has been transmitted, pop the element
      // and reset the config_index to the beginning of the
      // config values
      if( config_index == static_cast<int>(MOTOR_CONFIG_CRC) )
      {
	//loop on all the motors and send a CRC of 0
        // except for the motor we're reconfiguring
        for( unsigned int i = 0 ; i < NUM_MOTORS ; ++i )
        {
          if( i != motor_index )
            command->motor_data[i] = 0;
        }

        //reset the config_index and remove the configuration
        // we just sent from the configurations queue
        reconfig_queue.pop();
        config_index = MOTOR_CONFIG_FIRST_VALUE;

      }
      ++config_index;

    } //endelse reconfig_queue.size()
  }


  void SrRobotLib::calibrate_joint(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp)
  {
    actuator->state_.raw_sensor_values_.clear();
    actuator->state_.calibrated_sensor_values_.clear();

    if(joint_tmp->joint_to_sensor.calibrate_after_combining_sensors)
    {
      //first we combine the different sensors and then we
      // calibrate the value we obtained. This is used for
      // some compound sensors ( THJ5 = cal(THJ5A + THJ5B))
      double raw_position = 0.0;
      //when combining the values, we use the coefficient imported
      // from the sensor_to_joint.yaml file (in sr_edc_launch/config)
      BOOST_FOREACH(shadow_joints::PartialJointToSensor joint_to_sensor, joint_tmp->joint_to_sensor.joint_to_sensor_vector)
      {
        int tmp_raw = status_data->sensors[joint_to_sensor.sensor_id];
        actuator->state_.raw_sensor_values_.push_back( tmp_raw );
        raw_position += static_cast<double>(tmp_raw)*joint_to_sensor.coeff;
      }

      //and now we calibrate
      calibration_tmp = calibration_map.find(joint_tmp->joint_name);
      actuator->state_.position_ = calibration_tmp->compute( static_cast<double>(raw_position) );
    }
    else
    {
      //we calibrate the different sensors first and we combine the calibrated
      //values. This is used in the joint 0s for example ( J0 = cal(J1)+cal(J2) )
      double calibrated_position = 0.0;
      shadow_joints::PartialJointToSensor joint_to_sensor;
      std::string sensor_name;

      ROS_DEBUG_STREAM("Combining actuator " << joint_tmp->joint_name);

      for(unsigned int index_joint_to_sensor=0;
          index_joint_to_sensor < joint_tmp->joint_to_sensor.joint_to_sensor_vector.size();
          ++index_joint_to_sensor)
      {
        joint_to_sensor = joint_tmp->joint_to_sensor.joint_to_sensor_vector[index_joint_to_sensor];
        sensor_name = joint_tmp->joint_to_sensor.sensor_names[index_joint_to_sensor];

        //get the raw position
        int raw_pos = status_data->sensors[joint_to_sensor.sensor_id];
        //push the new raw values
        actuator->state_.raw_sensor_values_.push_back( raw_pos );

        //calibrate and then combine
        calibration_tmp = calibration_map.find(sensor_name);
        double tmp_cal_value = calibration_tmp->compute( static_cast<double>( raw_pos) );

        //push the new calibrated values.
        actuator->state_.calibrated_sensor_values_.push_back( tmp_cal_value );

        calibrated_position += tmp_cal_value * joint_to_sensor.coeff;

        ROS_DEBUG_STREAM("      -> "<< sensor_name<< " raw = " << raw_pos
                         << " calibrated = " << calibrated_position);
      }
      actuator->state_.position_ = calibrated_position;

      ROS_DEBUG_STREAM("          => "<< actuator->state_.position_);
    }
  } //end calibrate_joint()

  void SrRobotLib::read_additional_data(boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp)
  {
    /*
      if (motor_index_full & 1)
      {
      if (status_data->which_motor_data_arrived == 8)
      {
      ROS_ERROR_STREAM ("[*] " << status_data->motor_data_type);
      }
      else
      {
      ROS_ERROR_STREAM ("[ ] " << status_data->motor_data_type);
      }
      ROS_ERROR_STREAM("["<< joint_tmp->joint_name << "] : incoming mask=" << status_data->which_motor_data_arrived
      << " / bad data mask : " << status_data->which_motor_data_had_errors
      << " / motor_index: " << motor_index_full << " / "<< index_motor_in_msg);
      }
    */


    //check the masks to see if the CAN messages arrived to the motors
    //the flag should be set to 1 for each motor
    joint_tmp->motor->motor_ok = sr_math_utils::is_bit_mask_index_true(status_data->which_motor_data_arrived, motor_index_full);

    //check the masks to see if a bad CAN message arrived
    //the flag should be 0
    joint_tmp->motor->bad_data = sr_math_utils::is_bit_mask_index_true(status_data->which_motor_data_had_errors, index_motor_in_msg);

    crc_unions::union16 tmp_value;

    if(joint_tmp->motor->motor_ok && !(joint_tmp->motor->bad_data) )
    {
      //TODO: Hugo: how can I get the correct values from the motor???

      //we received the data and it was correct
      switch(status_data->motor_data_type)
      {
      case MOTOR_DATA_SGL:
	if(motor_index_full == 9)
        {
          msg_debug.data = status_data->motor_data_packet[index_motor_in_msg].misc;
          debug_publishers[0].publish(msg_debug);
        }

        actuator->state_.strain_gauge_left_ =  status_data->motor_data_packet[index_motor_in_msg].misc;
        break;
      case MOTOR_DATA_SGR:
	if(motor_index_full == 9)
        {
          msg_debug.data = status_data->motor_data_packet[index_motor_in_msg].misc;
          debug_publishers[1].publish(msg_debug);
        }

        actuator->state_.strain_gauge_right_ =  status_data->motor_data_packet[index_motor_in_msg].misc;
        break;
      case MOTOR_DATA_PWM:
        actuator->state_.last_executed_effort_ =  static_cast<double>(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_FLAGS:
        actuator->state_.flags_ = humanize_flags(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_CURRENT:
        //we're receiving the current in milli amps
        actuator->state_.last_measured_current_ = static_cast<double>(status_data->motor_data_packet[index_motor_in_msg].misc)/1000.0;
        break;
      case MOTOR_DATA_VOLTAGE:
        actuator->state_.motor_voltage_ = static_cast<double>(status_data->motor_data_packet[index_motor_in_msg].misc ) / 256.0;
        break;
      case MOTOR_DATA_TEMPERATURE:
        actuator->state_.temperature_ = static_cast<double>(status_data->motor_data_packet[index_motor_in_msg].misc) / 256.0;
        break;
      case MOTOR_DATA_CAN_NUM_RECEIVED:
        actuator->state_.can_msgs_received_ = status_data->motor_data_packet[index_motor_in_msg].misc;
        break;
      case MOTOR_DATA_CAN_NUM_TRANSMITTED:
        actuator->state_.can_msgs_transmitted_ = status_data->motor_data_packet[index_motor_in_msg].misc;
        break;
      case MOTOR_DATA_SVN_REVISION:
        actuator->state_.server_firmware_svn_revision_ = status_data->motor_data_packet[index_motor_in_msg].torque;
        //the bit 15 tells us if the firmware version on the motor is a modified version of the svn.
        actuator->state_.firmware_modified_ = ( (status_data->motor_data_packet[index_motor_in_msg].misc & 0x8000) != 0 );
        // the other 14 bits are the svn revision currently programmed on the pic
        actuator->state_.pic_firmware_svn_revision_ = ( status_data->motor_data_packet[index_motor_in_msg].misc & 0x7FFF );
        break;
      case MOTOR_DATA_CAN_ERROR_COUNTERS:
        actuator->state_.tests_ = status_data->motor_data_packet[index_motor_in_msg].misc;
        break;
      case MOTOR_DATA_F_P:
        actuator->state_.force_control_f_ = status_data->motor_data_packet[index_motor_in_msg].torque;
        actuator->state_.force_control_p_ = status_data->motor_data_packet[index_motor_in_msg].misc;
        break;
      case MOTOR_DATA_I_D:
        actuator->state_.force_control_i_ = status_data->motor_data_packet[index_motor_in_msg].torque;
        actuator->state_.force_control_d_ = status_data->motor_data_packet[index_motor_in_msg].misc;
        break;
      case MOTOR_DATA_IMAX_DEADBAND_SIGN:
        actuator->state_.force_control_imax_ = status_data->motor_data_packet[index_motor_in_msg].torque;

        tmp_value.word = status_data->motor_data_packet[index_motor_in_msg].misc;
        actuator->state_.force_control_deadband_ = static_cast<int>(tmp_value.byte[0]);
        //how do I read the sign?
        actuator->state_.force_control_sign_ = static_cast<int>(tmp_value.byte[1]);
        break;
      default:
        break;
      }

      actuator->state_.last_measured_effort_ = static_cast<double>(status_data->motor_data_packet[index_motor_in_msg].torque);
    }
  }

  std::vector<std::pair<std::string, bool> > SrRobotLib::humanize_flags(int flag)
  {
    std::vector<std::pair<std::string, bool> > flags;

    //16 is the number of flags
    for(unsigned int i = 0; i < 16; ++i)
    {
      std::pair<std::string, bool> new_flag;
      //if the flag is set add the name
      if( sr_math_utils::is_bit_mask_index_true(flag, i) )
      {
        if( sr_math_utils::is_bit_mask_index_true(SERIOUS_ERROR_FLAGS, i) )
          new_flag.second = true;
        else
          new_flag.second = false;

        new_flag.first = error_flag_names[i];
        flags.push_back( new_flag );
      }
    }
    return flags;
  }

  void SrRobotLib::generate_force_control_config(int motor_index, int sg_left, int sg_right,
                                                 int f, int p, int i, int d, int imax,
                                                 int deadband, int sign)
  {
    //the vector is of the size of the TO_MOTOR_DATA_TYPE enum.
    //the value of the element at a given index is the value
    //for the given MOTOR_CONFIG.
    std::vector<crc_unions::union16> full_config(MOTOR_CONFIG_CRC + 1);
    crc_unions::union16 value;

    //TODO: get each strain gauge amplifier and combine them here
    value.byte[0] = sg_left;
    value.byte[1] = sg_right;
    full_config.at(MOTOR_CONFIG_SG_REFS) = value;

    value.word = f;
    full_config.at(MOTOR_CONFIG_F) = value;

    value.word = p;
    full_config.at(MOTOR_CONFIG_P) = value;

    value.word = i;
    full_config.at(MOTOR_CONFIG_I) = value;

    value.word = d;
    full_config.at(MOTOR_CONFIG_D) = value;

    value.word = imax;
    full_config.at(MOTOR_CONFIG_IMAX) = value;

    value.byte[0] = deadband;
    value.byte[1] = sign;
    full_config.at(MOTOR_CONFIG_DEADBAND_SIGN) = value;


    //compute crc
    crc_result = 0;
    for(unsigned int i = MOTOR_CONFIG_FIRST_VALUE; i <= MOTOR_CONFIG_LAST_VALUE; ++i)
    {
      crc_byte = full_config.at(i).byte[0];
      INSERT_CRC_CALCULATION_HERE;

      crc_byte = full_config.at(i).byte[1];
      INSERT_CRC_CALCULATION_HERE;
    }

    //never send a CRC of 0, send 1 if the
    // computed CRC is 0 (0 is a code for
    // ignoring the config)
    if( crc_result == 0 )
      crc_result = 1;
    value.word = crc_result;
    full_config.at(MOTOR_CONFIG_CRC) = value;

    ForceConfig config;
    config.first = motor_index;
    config.second = full_config;
    //push the new config to the configuration queue
    reconfig_queue.push(config);
  }
} //end namespace

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
