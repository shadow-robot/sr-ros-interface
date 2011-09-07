/**
 * @file   sr_hand_lib.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, <contact@shadowrobot.com>
 * @date   Fri Jun  3 13:05:10 2011
 *
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
 * @brief This is a library for the etherCAT hand.
 * You can find it instantiated in the sr_edc_ethercat_drivers.
 *
 *
 */

#include "sr_robot_lib/sr_hand_lib.hpp"
#include <algorithm>
#include <string>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <sr_utilities/sr_math_utils.hpp>

namespace shadow_robot
{
  const int SrHandLib::nb_motor_data = 14;
  const char* SrHandLib::human_readable_motor_data_types[nb_motor_data] = {"sgl", "sgr", "pwm", "flags", "current",
                                                                           "voltage", "temperature", "can_num_received",
                                                                           "can_num_transmitted", "slow_data",
                                                                           "can_error_counters",
                                                                           "pterm", "iterm", "dterm"};

  const FROM_MOTOR_DATA_TYPE SrHandLib::motor_data_types[nb_motor_data] = {MOTOR_DATA_SGL, MOTOR_DATA_SGR,
                                                                           MOTOR_DATA_PWM, MOTOR_DATA_FLAGS,
                                                                           MOTOR_DATA_CURRENT, MOTOR_DATA_VOLTAGE,
                                                                           MOTOR_DATA_TEMPERATURE, MOTOR_DATA_CAN_NUM_RECEIVED,
                                                                           MOTOR_DATA_CAN_NUM_TRANSMITTED, MOTOR_DATA_SLOW_MISC,
                                                                           MOTOR_DATA_CAN_ERROR_COUNTERS,
                                                                           MOTOR_DATA_PTERM, MOTOR_DATA_ITERM,
                                                                           MOTOR_DATA_DTERM};
  const unsigned int SrRobotLib::nb_tactiles = 5;

  SrHandLib::SrHandLib(pr2_hardware_interface::HardwareInterface *hw) :
    SrRobotLib(hw)
  {
    //read the motor polling frequency from the parameter server
    std::vector<motor_updater::UpdateConfig> update_rate_configs_vector = read_update_rate_configs();
    motor_updater_ = boost::shared_ptr<motor_updater::MotorUpdater>(new motor_updater::MotorUpdater(update_rate_configs_vector));

    //TODO: read this from config/EEProm?
    std::vector<shadow_joints::JointToSensor > joint_to_sensor_vect = read_joint_to_sensor_mapping();

    //initializing the joints vector
    std::vector<std::string> joint_names_tmp;
    std::vector<int> motor_ids = read_joint_to_motor_mapping();
    std::vector<shadow_joints::JointToSensor > joints_to_sensors;
    std::vector<sr_actuator::SrActuator*> actuators;

    ROS_ASSERT(motor_ids.size() == JOINTS_NUM_0220);
    ROS_ASSERT(joint_to_sensor_vect.size() == JOINTS_NUM_0220);

    for(unsigned int i=0; i< JOINTS_NUM_0220; ++i)
    {
      joint_names_tmp.push_back(std::string(joint_names[i]));
      shadow_joints::JointToSensor tmp_jts = joint_to_sensor_vect[i];
      joints_to_sensors.push_back(tmp_jts);

      //initializing the actuators.
      sr_actuator::SrActuator* actuator = new sr_actuator::SrActuator(joint_names[i]);
      ROS_INFO_STREAM("adding actuator: "<<joint_names[i]);
      actuators.push_back( actuator );

      if(hw)
      {
        if(!hw->addActuator(actuator) )
        {
          ROS_FATAL("An actuator of the name '%s' already exists.", actuator->name_.c_str());
        }
      }
    }
    initialize(joint_names_tmp, motor_ids, joint_to_sensor_vect, actuators);

    //initialize the calibration map
    this->calibration_map = read_joint_calibration();

    //initialize the vector of tactiles
    for(unsigned int i=0; i < nb_tactiles; ++i)
    {
      tactiles_vector.push_back( new TACTILE_SENSOR_STATUS() );
    }
#ifdef DEBUG_PUBLISHER
    //advertise the debug service, used to set which data we want to publish on the debug topics
    debug_service = nh_tilde.advertiseService( "set_debug_publishers", &SrHandLib::set_debug_data_to_publish, this);
#endif
  }

  SrHandLib::~SrHandLib()
  {
    boost::ptr_vector<shadow_joints::Joint>::iterator joint = joints_vector.begin();
    for(;joint != joints_vector.end(); ++joint)
    {
      delete joint->motor->actuator;
    }
  }

  void SrHandLib::initialize(std::vector<std::string> joint_names,
                             std::vector<int> motor_ids,
                             std::vector<shadow_joints::JointToSensor> joint_to_sensors,
                             std::vector<sr_actuator::SrActuator*> actuators)
  {
    for(unsigned int index = 0; index < joint_names.size(); ++index)
    {
      //add the joint and the vector of joints.
      joints_vector.push_back( new shadow_joints::Joint() );

      //get the last inserted joint
      boost::ptr_vector<shadow_joints::Joint>::reverse_iterator joint = joints_vector.rbegin();

      //update the joint variables
      joint->joint_name = joint_names[index];
      joint->joint_to_sensor = joint_to_sensors[index];

      if(motor_ids[index] == -1) //no motor associated to this joint
        joint->has_motor = false;
      else
        joint->has_motor = true;

      joint->motor    = boost::shared_ptr<shadow_joints::Motor>( new shadow_joints::Motor() );
      joint->motor->motor_id = motor_ids[index];
      joint->motor->actuator = actuators[index];

      std::stringstream ss;
      ss << "change_force_PID_" << joint_names[index];
      //initialize the force pid service
      joint->motor->force_pid_service = nh_tilde.advertiseService<sr_robot_msgs::ForceController::Request,sr_robot_msgs::ForceController::Response>( ss.str().c_str(),
                                                                                                                                                     boost::bind( &SrHandLib::force_pid_callback, this, _1, _2, joint->motor->motor_id) );

      ss.str("");
      ss << "reset_motor_" << joint_names[index];
      //initialize the reset motor service
      joint->motor->reset_motor_service = nh_tilde.advertiseService<std_srvs::Empty::Request,std_srvs::Empty::Response>( ss.str().c_str(),
                                                                                                                         boost::bind( &SrHandLib::reset_motor_callback, this, _1, _2, std::pair<int,std::string>(joint->motor->motor_id, joint->joint_name) ) );

    } //end for joints.
  }

  bool SrHandLib::reset_motor_callback(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response,
                                       std::pair<int,std::string> joint)
  {
    reset_motors_queue.push(joint.first);

    //wait for the reset to be sent
    sleep(2.0);

    //then reset the pids
    resend_pids(joint.second, joint.first);

    return true;
  }


  void SrHandLib::resend_pids(std::string joint_name, int motor_index)
  {
    //read the parameters from the parameter server and set the pid
    // values.
    std::stringstream full_param;

    int f, p, i, d, imax, max_pwm, sg_left, sg_right, deadband, sign;
    std::string act_name = boost::to_lower_copy(joint_name);

    full_param << "/" << act_name << "/pid/f";
    nodehandle_.param<int>(full_param.str(), f, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/p";
    nodehandle_.param<int>(full_param.str(), p, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/i";
    nodehandle_.param<int>(full_param.str(), i, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/d";
    nodehandle_.param<int>(full_param.str(), d, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/imax";
    nodehandle_.param<int>(full_param.str(), imax, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/max_pwm";
    nodehandle_.param<int>(full_param.str(), max_pwm, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/sg_left";
    nodehandle_.param<int>(full_param.str(), sg_left, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/sg_right";
    nodehandle_.param<int>(full_param.str(), sg_right, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/deadband";
    nodehandle_.param<int>(full_param.str(), deadband, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/sign";
    nodehandle_.param<int>(full_param.str(), sign, 0);
    full_param.str("");

    sr_robot_msgs::ForceController::Request pid_request;
    pid_request.maxpwm = max_pwm;
    pid_request.sgleftref = sg_left;
    pid_request.sgrightref = sg_right;
    pid_request.f = f;
    pid_request.p = p;
    pid_request.i = i;
    pid_request.d = d;
    pid_request.imax = imax;
    pid_request.deadband = deadband;
    pid_request.sign = sign;
    sr_robot_msgs::ForceController::Response pid_response;
    if( force_pid_callback(pid_request, pid_response, motor_index ) )
    {
      return;
    }

    ROS_WARN_STREAM( "Didn't load the force pid settings for the motor in joint " << act_name );
  }


  bool SrHandLib::force_pid_callback(sr_robot_msgs::ForceController::Request& request,
                                     sr_robot_msgs::ForceController::Response& response,
                                     int motor_index)
  {
    ROS_INFO_STREAM("Received new force PID parameters for motor " << motor_index);

    //Check the parameters are in the correct ranges
    if( motor_index > 20 )
    {
      ROS_WARN_STREAM(" Wrong motor index specified: " << motor_index);
      response.configured = false;;
      return false;
    }

    if( !( (request.maxpwm >= MOTOR_DEMAND_PWM_RANGE_MIN) &&
           (request.maxpwm <= MOTOR_DEMAND_PWM_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter maxpwm is out of range : " << request.maxpwm << " -> not in [" <<
                      MOTOR_DEMAND_PWM_RANGE_MIN << " ; " << MOTOR_DEMAND_PWM_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.f >= MOTOR_CONFIG_F_RANGE_MIN) &&
           (request.maxpwm <= MOTOR_CONFIG_F_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter f is out of range : " << request.f << " -> not in [" <<
                      MOTOR_CONFIG_F_RANGE_MIN << " ; " << MOTOR_CONFIG_F_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.p >= MOTOR_CONFIG_P_RANGE_MIN) &&
           (request.p <= MOTOR_CONFIG_P_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter p is out of range : " << request.p << " -> not in [" <<
                      MOTOR_CONFIG_P_RANGE_MIN << " ; " << MOTOR_CONFIG_P_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.i >= MOTOR_CONFIG_I_RANGE_MIN) &&
           (request.i <= MOTOR_CONFIG_I_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter i is out of range : " << request.i << " -> not in [" <<
                      MOTOR_CONFIG_I_RANGE_MIN << " ; " << MOTOR_CONFIG_I_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.d >= MOTOR_CONFIG_D_RANGE_MIN) &&
           (request.d <= MOTOR_CONFIG_D_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter d is out of range : " << request.d << " -> not in [" <<
                      MOTOR_CONFIG_D_RANGE_MIN << " ; " << MOTOR_CONFIG_D_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.imax >= MOTOR_CONFIG_IMAX_RANGE_MIN) &&
           (request.imax <= MOTOR_CONFIG_IMAX_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter imax is out of range : " << request.imax << " -> not in [" <<
                      MOTOR_CONFIG_IMAX_RANGE_MIN << " ; " << MOTOR_CONFIG_IMAX_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.deadband >= MOTOR_CONFIG_DEADBAND_RANGE_MIN) &&
           (request.deadband <= MOTOR_CONFIG_DEADBAND_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter deadband is out of range : " << request.deadband << " -> not in [" <<
                      MOTOR_CONFIG_DEADBAND_RANGE_MIN << " ; " << MOTOR_CONFIG_DEADBAND_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    if( !( (request.sign >= MOTOR_CONFIG_SIGN_RANGE_MIN) &&
           (request.sign <= MOTOR_CONFIG_SIGN_RANGE_MAX) )
      )
    {
      ROS_WARN_STREAM(" pid parameter sign is out of range : " << request.sign << " -> not in [" <<
                      MOTOR_CONFIG_SIGN_RANGE_MIN << " ; " << MOTOR_CONFIG_SIGN_RANGE_MAX << "]");
      response.configured = false;
      return false;
    }

    //ok, the parameters sent are coherent, send the demand to the motor.
    generate_force_control_config( motor_index, request.maxpwm, request.sgleftref,
                                   request.sgrightref, request.f, request.p, request.i,
                                   request.d, request.imax, request.deadband, request.sign );
    response.configured = true;
    return true;
  }


  std::vector<shadow_joints::JointToSensor> SrHandLib::read_joint_to_sensor_mapping()
  {
    std::vector<shadow_joints::JointToSensor> joint_to_sensor_vect;

    std::map<std::string, int> sensors_map;
    for(unsigned int i=0; i < SENSORS_NUM_0220; ++i)
    {
      sensors_map[ sensor_names[i] ] = i;
    }

    XmlRpc::XmlRpcValue joint_to_sensor_mapping;
    nodehandle_.getParam("joint_to_sensor_mapping", joint_to_sensor_mapping);
    ROS_ASSERT(joint_to_sensor_mapping.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < joint_to_sensor_mapping.size(); ++i)
    {
      shadow_joints::JointToSensor tmp_vect;

      XmlRpc::XmlRpcValue map_one_joint = joint_to_sensor_mapping[i];

      //The parameter can either start by an array (sensor_name, coeff)
      // or by an integer to specify if we calibrate before combining
      // the different sensors
      int param_index = 0;
      //Check if the calibrate after combine int is set to 1
      if(map_one_joint[param_index].getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        if(1 == static_cast<int>(map_one_joint[0]) )
          tmp_vect.calibrate_after_combining_sensors = true;
        else
          tmp_vect.calibrate_after_combining_sensors = false;

        param_index ++;
      }
      else //by default we calibrate before combining the sensors
        tmp_vect.calibrate_after_combining_sensors = false;

      ROS_ASSERT(map_one_joint.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int32_t i = param_index; i < map_one_joint.size(); ++i)
      {
        ROS_ASSERT(map_one_joint[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
        shadow_joints::PartialJointToSensor tmp_joint_to_sensor;

        ROS_ASSERT(map_one_joint[i][0].getType() == XmlRpc::XmlRpcValue::TypeString);
        tmp_vect.sensor_names.push_back( static_cast<std::string>(map_one_joint[i][0]) );
        tmp_joint_to_sensor.sensor_id = sensors_map[ static_cast<std::string>(map_one_joint[i][0]) ];

        ROS_ASSERT(map_one_joint[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        tmp_joint_to_sensor.coeff = static_cast<double> (map_one_joint[i][1]);
        tmp_vect.joint_to_sensor_vector.push_back(tmp_joint_to_sensor);
      }
      joint_to_sensor_vect.push_back(tmp_vect);
    }

    return joint_to_sensor_vect;
  } //end read_joint_to_sensor_mapping



  shadow_joints::CalibrationMap SrHandLib::read_joint_calibration()
  {
    shadow_joints::CalibrationMap joint_calibration;
    std::string param_name = "sr_calibrations";

    XmlRpc::XmlRpcValue calib;
    nodehandle_.getParam(param_name, calib);
    ROS_ASSERT(calib.getType() == XmlRpc::XmlRpcValue::TypeArray);
    //iterate on all the joints
    for(int32_t index_cal = 0; index_cal < calib.size(); ++index_cal)
    {
      //check the calibration is well formatted:
      // first joint name, then calibration table
      ROS_ASSERT(calib[index_cal][0].getType() == XmlRpc::XmlRpcValue::TypeString);
      ROS_ASSERT(calib[index_cal][1].getType() == XmlRpc::XmlRpcValue::TypeArray);

      std::string joint_name = static_cast<std::string> (calib[index_cal][0]);
      std::vector<joint_calibration::Point> calib_table_tmp;

      //now iterates on the calibration table for the current joint
      for(int32_t index_table=0; index_table < calib[index_cal][1].size(); ++index_table)
      {
        ROS_ASSERT(calib[index_cal][1][index_table].getType() == XmlRpc::XmlRpcValue::TypeArray);
        //only 2 values per calibration point: raw and calibrated (doubles)
        ROS_ASSERT(calib[index_cal][1][index_table].size() == 2);
        ROS_ASSERT(calib[index_cal][1][index_table][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(calib[index_cal][1][index_table][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);


        joint_calibration::Point point_tmp;
        point_tmp.raw_value = static_cast<double> (calib[index_cal][1][index_table][0]);
        point_tmp.calibrated_value = sr_math_utils::to_rad( static_cast<double> (calib[index_cal][1][index_table][1]) );
        calib_table_tmp.push_back(point_tmp);
      }

      joint_calibration.insert(joint_name, boost::shared_ptr<shadow_robot::JointCalibration>(new shadow_robot::JointCalibration(calib_table_tmp)) );
    }

    return joint_calibration;
  } //end read_joint_calibration


  std::vector<int> SrHandLib::read_joint_to_motor_mapping()
  {
    std::vector<int> motor_ids;
    std::string param_name = "joint_to_motor_mapping";

    XmlRpc::XmlRpcValue mapping;
    nodehandle_.getParam(param_name, mapping);
    ROS_ASSERT(mapping.getType() == XmlRpc::XmlRpcValue::TypeArray);
    //iterate on all the joints
    for(int32_t i = 0; i < mapping.size(); ++i)
    {
      ROS_ASSERT(mapping[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
      motor_ids.push_back(static_cast<int>(mapping[i]));
    }

    return motor_ids;
  } //end read_joint_to_motor_mapping


  std::vector<motor_updater::UpdateConfig> SrHandLib::read_update_rate_configs()
  {
    std::vector<motor_updater::UpdateConfig> update_rate_configs_vector;
    std::string base_param = "motor_data_update_rate/";
    typedef std::pair<std::string, FROM_MOTOR_DATA_TYPE> ConfPair;
    std::vector<ConfPair> config;

    for(int i=0; i<nb_motor_data; ++i)
    {
      ConfPair tmp;

      ROS_DEBUG_STREAM(" read update rate config [" << i<< "] = "  << human_readable_motor_data_types[i]);

      tmp.first = base_param + human_readable_motor_data_types[i];
      tmp.second = motor_data_types[i];
      config.push_back(tmp);
    }

    for(unsigned int i = 0; i < config.size(); ++i)
    {
      double rate;
      nodehandle_.getParam(config[i].first, rate);
      motor_updater::UpdateConfig config_tmp;

      config_tmp.when_to_update = rate;
      config_tmp.what_to_update = config[i].second;
      update_rate_configs_vector.push_back(config_tmp);
    }

    return update_rate_configs_vector;
  }

#ifdef DEBUG_PUBLISHER
  bool SrHandLib::set_debug_data_to_publish(sr_robot_msgs::SetDebugData::Request& request,
                                            sr_robot_msgs::SetDebugData::Response& response)
  {
    //check if the publisher_index is correct
    if( request.publisher_index < nb_debug_publishers_const )
    {
      if( request.motor_index > NUM_MOTORS )
      {
        response.success = false;
        return false;
      }
      if( request.motor_data_type > 0 )
      {
        if( (request.motor_data_type < MOTOR_DATA_SGL) ||
            (request.motor_data_type > MOTOR_DATA_DTERM) )
        {
          response.success = false;
          return false;
        }
      }
      if(!debug_mutex.timed_lock(boost::posix_time::microseconds(debug_mutex_lock_wait_time)))
      {
        response.success = false;
        return false;
      }

      debug_motor_indexes_and_data[request.publisher_index] = boost::shared_ptr<std::pair<int, int> >(new std::pair<int, int>());

      debug_motor_indexes_and_data[request.publisher_index]->first = request.motor_index;
      debug_motor_indexes_and_data[request.publisher_index]->second = request.motor_data_type;
      debug_mutex.unlock();
    }
    else
    {
      response.success = false;
      return false;
    }

    response.success = true;
    return true;
  }
#endif

}// end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
