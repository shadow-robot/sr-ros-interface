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

#include <sys/time.h>

#include <sr_utilities/sr_math_utils.hpp>

#include <ros/ros.h>

namespace shadow_robot
{
#ifdef DEBUG_PUBLISHER
  //max of 20 publishers for debug
  const int SrRobotLib::nb_debug_publishers_const = 20;
  const int SrRobotLib::debug_mutex_lock_wait_time = 100;
#endif
  const int SrRobotLib::number_of_positions_to_keep = 5;
  const int SrRobotLib::number_of_positions_for_filter = 2;

  SrRobotLib::SrRobotLib(pr2_hardware_interface::HardwareInterface *hw)
    : main_pic_idle_time(0), main_pic_idle_time_min(1000), config_index(MOTOR_CONFIG_FIRST_VALUE), nh_tilde("~"),
      last_can_msgs_received(0), last_can_msgs_transmitted(0)
  {
#ifdef DEBUG_PUBLISHER
    debug_motor_indexes_and_data.resize(nb_debug_publishers_const);
    for( int i = 0; i < nb_debug_publishers_const ; ++i )
    {
      std::stringstream ss;
      ss << "srh/debug_" << i;
      debug_publishers.push_back(node_handle.advertise<std_msgs::Int16>(ss.str().c_str(),100));
    }
#endif
  }


  void SrRobotLib::update(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS* status_data)
  {
    this->status_data = status_data;

    //read the PIC idle time
    main_pic_idle_time = status_data->idle_time_us;
    if( status_data->idle_time_us < main_pic_idle_time_min )
      main_pic_idle_time_min = status_data->idle_time_us;

    //get the current timestamp
    struct timeval tv;
    double timestamp = 0.0;
    if (gettimeofday(&tv, NULL))
    {
      ROS_WARN("SrRobotLib: Failed to get system time, timestamp in state will be zero");
    }
    else
    {
      timestamp = double(tv.tv_sec) + double(tv.tv_usec) / 1.0e+6;
    }
    //First we read the joints informations
    boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp = joints_vector.begin();
    for(;joint_tmp != joints_vector.end(); ++joint_tmp)
    {
      actuator = (joint_tmp->motor->actuator);

      motor_index_full = joint_tmp->motor->motor_id;
      actuator->state_.is_enabled_ = 1;
      actuator->state_.device_id_ = motor_index_full;
      actuator->state_.halted_ = false;

      //calibrate the joint and update the position.
      calibrate_joint(joint_tmp);

      //add the last position to the queue
      std::pair<double, double> pos_and_time;
      joint_tmp->motor->actuator->state_.timestamp_ = timestamp;
      pos_and_time.first = joint_tmp->motor->actuator->state_.position_;
      pos_and_time.second = timestamp;
      joint_tmp->last_positions.push_back( pos_and_time );

      //If we have more than N values, filter the position:
      //    compute the average of the last N values
      int queue_size = joint_tmp->last_positions.size();
      if(queue_size > number_of_positions_for_filter)
      {
        double average = 0.0;
        //compute the average of the last N values
        for(int i=queue_size - 1; i > queue_size - number_of_positions_for_filter; --i)
          average += joint_tmp->last_positions[i].first;
        average /= number_of_positions_for_filter;

        //reset the position to the filtered value
        joint_tmp->motor->actuator->state_.position_ = average;

        //update the position queue with the filtered value
        pos_and_time.first = average;
      }
      //compute the velocity
      joint_tmp->motor->actuator->state_.velocity_ = (joint_tmp->last_positions.front().first - joint_tmp->last_positions.back().first) / (joint_tmp->last_positions.front().second - joint_tmp->last_positions.back().second);
      if(queue_size > number_of_positions_to_keep)
        joint_tmp->last_positions.pop_front();

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
    } //end for joint

    //then we read the tactile sensors information
    tactile_data_valid = static_cast<int16u>(status_data->tactile_data_valid);
    //TODO: use memcopy instead?
    //FF
    for( unsigned int id_data = 0; id_data < 8; ++id_data)
    {
      tactiles_vector[0].data[id_data] = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[2].data[id_data]) );
    }
    //TH
    for( unsigned int id_data = 0; id_data < 8; ++id_data)
    {
      tactiles_vector[4].data[id_data] = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[4].data[id_data]) );
    }
    //LF
    for( unsigned int id_data = 0; id_data < 8; ++id_data)
    {
      tactiles_vector[3].data[id_data] = static_cast<unsigned int>(static_cast<int16u>(status_data->tactile[3].data[id_data]) );
    }


  } //end update()

  void SrRobotLib::build_motor_command(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND* command)
  {
    motor_updater_->build_update_motor_command(command);

    ///////
    // Now we chose the command to send to the motor
    // by default we send a torque demand (we're running
    // the force control on the motors), but if we have a waiting
    // configuration or a reset command, then we send the configuration
    // or the reset.
    if( reconfig_queue.empty() && reset_motors_queue.empty() )
    {
      //no config to send
      command->to_motor_data_type   = MOTOR_DEMAND_TORQUE;

      //loop on all the joints and update their motor: we're sending commands to all the motors.
      boost::ptr_vector<shadow_joints::Joint>::iterator joint_tmp = joints_vector.begin();
      for(;joint_tmp != joints_vector.end(); ++joint_tmp)
      {
        if(joint_tmp->has_motor)
        {
	  //ROS_ERROR_STREAM(" LFJ3 "<< joint_tmp->motor->actuator->command_.effort_);
          command->motor_data[joint_tmp->motor->motor_id] = joint_tmp->motor->actuator->command_.effort_;

#ifdef DEBUG_PUBLISHER
          //publish the debug values for the given motors.
          // NB: debug_motor_indexes_and_data is smaller
          //     than debug_publishers.
          int publisher_index = 0;
          boost::shared_ptr<std::pair<int,int> > debug_pair;
          if( debug_mutex.try_lock() )
          {
            BOOST_FOREACH(debug_pair, debug_motor_indexes_and_data)
            {
              if( debug_pair != NULL )
              {
                //check if we want to publish some data for the current motor
                if( debug_pair->first == joint_tmp->motor->motor_id )
                {
                  //check if it's the correct data
                  if( debug_pair->second == -1 )
                  {
                    msg_debug.data =  joint_tmp->motor->actuator->command_.effort_;
                    debug_publishers[publisher_index].publish(msg_debug);
                  }
                }
              }
              publisher_index ++;
            }

            debug_mutex.unlock();
          } //end try_lock
#endif

          joint_tmp->motor->actuator->state_.last_commanded_effort_ = joint_tmp->motor->actuator->command_.effort_;
        } //end if has_motor
      } // end for each joint
    } //endif reconfig_queue.empty()
    else
    {
      if( !reset_motors_queue.empty() )
      {
        //we have some reset command waiting.
        // We'll send all of them
        command->to_motor_data_type = MOTOR_SYSTEM_RESET;

        while( !reset_motors_queue.empty() )
        {
          short motor_id = reset_motors_queue.front();
          reset_motors_queue.pop();

          // we send the MOTOR_RESET_SYSTEM_KEY
          // and the motor id (on the bus)
          crc_unions::union16 to_send;
          to_send.byte[1] = MOTOR_SYSTEM_RESET_KEY >> 8;
          if( motor_id > 9 )
            to_send.byte[0] = motor_id - 10;
          else
            to_send.byte[0] = motor_id;

          command->motor_data[motor_id] = to_send.word;
        }

      } // end if reset queue not empty
      else
      {
        if( !reconfig_queue.empty() )
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
            for( int i = 0 ; i < NUM_MOTORS ; ++i )
            {
              if( i != motor_index )
                command->motor_data[i] = 0;
            }

            //reset the config_index and remove the configuration
            // we just sent from the configurations queue
            reconfig_queue.pop();
            config_index = MOTOR_CONFIG_FIRST_VALUE;
          }
          else
            ++config_index;
        } //end if reconfig queue not empty
      } // end else reset_queue.empty
    } //endelse reconfig_queue.empty() && reset_queue.empty()
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
    //check the masks to see if the CAN messages arrived to the motors
    //the flag should be set to 1 for each motor
    joint_tmp->motor->motor_ok = sr_math_utils::is_bit_mask_index_true(status_data->which_motor_data_arrived, motor_index_full);

    //check the masks to see if a bad CAN message arrived
    //the flag should be 0
    joint_tmp->motor->bad_data = sr_math_utils::is_bit_mask_index_true(status_data->which_motor_data_had_errors, index_motor_in_msg);

    crc_unions::union16 tmp_value;

    if(joint_tmp->motor->motor_ok && !(joint_tmp->motor->bad_data) )
    {
#ifdef DEBUG_PUBLISHER
      int publisher_index = 0;
      //publish the debug values for the given motors.
      // NB: debug_motor_indexes_and_data is smaller
      //     than debug_publishers.
      boost::shared_ptr<std::pair<int,int> > debug_pair;

      if( debug_mutex.try_lock() )
      {
        BOOST_FOREACH(debug_pair, debug_motor_indexes_and_data)
        {
          if( debug_pair != NULL )
          {
            //check if we want to publish some data for the current motor
            if( debug_pair->first == joint_tmp->motor->motor_id )
            {
              //if < 0, then we're not asking for a FROM_MOTOR_DATA_TYPE
              if( debug_pair->second > 0 )
	      {
		//check if it's the correct data
		if( debug_pair->second == status_data->motor_data_type )
                {
                  msg_debug.data = status_data->motor_data_packet[index_motor_in_msg].misc;
                  debug_publishers[publisher_index].publish(msg_debug);
                }
	      }
            }
          }
          publisher_index ++;
        }

        debug_mutex.unlock();
      } //end try_lock
#endif

      //we received the data and it was correct
      bool read_torque = true;
      switch(status_data->motor_data_type)
      {
      case MOTOR_DATA_SGL:
        actuator->state_.strain_gauge_left_ = static_cast<int16s>( status_data->motor_data_packet[index_motor_in_msg].misc );
        break;
      case MOTOR_DATA_SGR:
        actuator->state_.strain_gauge_right_ =  static_cast<int16s>( status_data->motor_data_packet[index_motor_in_msg].misc );
        break;
      case MOTOR_DATA_PWM:
        actuator->state_.last_executed_effort_ =  static_cast<double>(static_cast<int16s>(status_data->motor_data_packet[index_motor_in_msg].misc));
        break;
      case MOTOR_DATA_FLAGS:
        actuator->state_.flags_ = humanize_flags(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_CURRENT:
        //we're receiving the current in milli amps
        actuator->state_.last_measured_current_ = static_cast<double>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc))/1000.0;
        break;
      case MOTOR_DATA_VOLTAGE:
        actuator->state_.motor_voltage_ = static_cast<double>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) ) / 256.0;
        break;
      case MOTOR_DATA_TEMPERATURE:
        actuator->state_.temperature_ = static_cast<double>(static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc)) / 256.0;
        break;
      case MOTOR_DATA_CAN_NUM_RECEIVED:
        // those are 16 bits values and will overflow -> we compute the real value.
        // This needs to be updated faster than the overflowing period (which should be roughly every 30s)
        actuator->state_.can_msgs_received_ = sr_math_utils::counter_with_overflow(actuator->state_.can_msgs_received_, last_can_msgs_received, static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc));
        last_can_msgs_received = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
        break;
      case MOTOR_DATA_CAN_NUM_TRANSMITTED:
        // those are 16 bits values and will overflow -> we compute the real value.
        // This needs to be updated faster than the overflowing period (which should be roughly every 30s)
        actuator->state_.can_msgs_transmitted_ = sr_math_utils::counter_with_overflow(actuator->state_.can_msgs_received_, last_can_msgs_received, static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
        last_can_msgs_transmitted = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
        break;

      case MOTOR_DATA_SLOW_MISC:
        //We received a slow data:
        // the slow data type is contained in .torque, while
        // the actual data is in .misc.
        switch( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].torque) )
        {
        case MOTOR_SLOW_DATA_SVN_REVISION:
          actuator->state_.pic_firmware_svn_revision_ = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_SVN_SERVER_REVISION:
          actuator->state_.server_firmware_svn_revision_ = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_SVN_MODIFIED:
          actuator->state_.firmware_modified_ = static_cast<bool>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_SERIAL_NUMBER_LOW:
          actuator->state_.serial_number_low = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_SERIAL_NUMBER_HIGH:
          actuator->state_.serial_number_high = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_GEAR_RATIO:
          actuator->state_.motor_gear_ratio = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_ASSEMBLY_DATE_YYYY:
          actuator->state_.assembly_data_year = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_ASSEMBLY_DATE_MMDD:
          actuator->state_.assembly_data_month = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc)/100 );
          actuator->state_.assembly_data_day = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) ) - actuator->state_.assembly_data_month;
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_F:
          actuator->state_.force_control_f_ = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_P:
          actuator->state_.force_control_p_ = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_I:
          actuator->state_.force_control_i_ = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_D:
          actuator->state_.force_control_d_ = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_IMAX:
          actuator->state_.force_control_imax_ = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_DEADSIGN:
          tmp_value.word = status_data->motor_data_packet[index_motor_in_msg].misc;
          actuator->state_.force_control_deadband_ = static_cast<int>(tmp_value.byte[0]);
          actuator->state_.force_control_sign_ = static_cast<int>(tmp_value.byte[1]);
          break;
        case MOTOR_SLOW_DATA_CONTROLLER_FREQUENCY:
          actuator->state_.force_control_frequency_ = static_cast<unsigned int>( static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc) );
          break;

        default:
          break;
        }
        break;

      case MOTOR_DATA_CAN_ERROR_COUNTERS:
        actuator->state_.can_error_counters = static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_PTERM:
	read_torque = false;
        actuator->state_.force_control_pterm = static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_ITERM:
	read_torque = false;
        actuator->state_.force_control_iterm = static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;
      case MOTOR_DATA_DTERM:
	read_torque = false;
        actuator->state_.force_control_dterm = static_cast<int16u>(status_data->motor_data_packet[index_motor_in_msg].misc);
        break;

      default:
        break;
      }

      if( read_torque )
	actuator->state_.last_measured_effort_ = static_cast<double>( static_cast<int16s>(status_data->motor_data_packet[index_motor_in_msg].torque) );
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

  void SrRobotLib::generate_force_control_config(int motor_index, int max_pwm, int sg_left, int sg_right,
                                                 int f, int p, int i, int d, int imax,
                                                 int deadband, int sign)
  {
    ROS_INFO_STREAM("Setting new pid values for motor" << motor_index << ": max_pwm="<< max_pwm
		    <<" sg_left=" << sg_left << " sg_right=" << sg_right << " f=" << f << " p="
		    << p << " i=" << i << " d="<< d << " imax=" << imax
		    << " deadband="<< deadband << " sign=" << sign);

    //the vector is of the size of the TO_MOTOR_DATA_TYPE enum.
    //the value of the element at a given index is the value
    //for the given MOTOR_CONFIG.
    std::vector<crc_unions::union16> full_config(MOTOR_CONFIG_CRC + 1);
    crc_unions::union16 value;

    value.word = max_pwm;
    full_config.at(MOTOR_CONFIG_MAX_PWM) = value;

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
    ROS_DEBUG_STREAM("deadband: " << static_cast<int>(static_cast<int8u>(value.byte[0]) ) << " value: " << static_cast<int16u>(value.word));


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
