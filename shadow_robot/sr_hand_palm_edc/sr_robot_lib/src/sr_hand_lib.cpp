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
#include <string>
#include <boost/foreach.hpp>

#include <stdio.h>

namespace shadow_robot
{
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
    std::vector<pr2_hardware_interface::Actuator*> actuators;

    ROS_ASSERT(motor_ids.size() == JOINTS_NUM_0220);
    ROS_ASSERT(joint_to_sensor_vect.size() == JOINTS_NUM_0220);

    for(unsigned int i=0; i< JOINTS_NUM_0220; ++i)
    {
      joint_names_tmp.push_back(std::string(joint_names[i]));
      shadow_joints::JointToSensor tmp_jts = joint_to_sensor_vect[i];
      joints_to_sensors.push_back(tmp_jts);

      //initializing the actuators.
      pr2_hardware_interface::Actuator* actuator = new pr2_hardware_interface::Actuator(joint_names[i]);
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
                             std::vector<pr2_hardware_interface::Actuator*> actuators)
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
    } //end for joints.
  }


  bool SrHandLib::force_pid_callback(sr_robot_msgs::ForceController::Request& request, sr_robot_msgs::ForceController::Response& response, int motor_index)
  {
    ROS_INFO_STREAM("Received new force PID parameters for motor " << motor_index);

    generate_force_control_config(request.sgleftref, request.sgrightref, request.f, request.p, request.i,
                                  request.d, request.imax, request.deadband, request.sign);

    ROS_WARN("Not implemented yet.");
    response.configured = true;
    return true;
  }


  std::vector<shadow_joints::JointToSensor> SrHandLib::read_joint_to_sensor_mapping()
  {
    std::vector<shadow_joints::JointToSensor> joint_to_sensor_vect;

    std::map<std::string, int> sensors_map;
    for(unsigned int i=0; i < SENSORS_NUM_0220; ++i)
    {
      sensors_map[sensor_names[i] ] = i;
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
        point_tmp.calibrated_value = static_cast<double> (calib[index_cal][1][index_table][1]);
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
    std::string base_topic = "motor_data_update_rate/";
    typedef std::pair<std::string, FROM_MOTOR_DATA_TYPE> ConfPair;
    std::vector<ConfPair> config;

    static const int nb_motor_data = 13;
    static const char* topics[nb_motor_data] = {"sgl", "sgr", "pwm", "flags", "current",
                                                "voltage", "temperature", "can_num_received",
                                                "can_num_transmitted", "svn_revision",
                                                "f_p", "i_d", "imax_deadband_sign"};

    static const FROM_MOTOR_DATA_TYPE data_types[nb_motor_data] = {MOTOR_DATA_SGL, MOTOR_DATA_SGR,
                                                                   MOTOR_DATA_PWM, MOTOR_DATA_FLAGS,
                                                                   MOTOR_DATA_CURRENT, MOTOR_DATA_VOLTAGE,
                                                                   MOTOR_DATA_TEMPERATURE, MOTOR_DATA_CAN_NUM_RECEIVED,
                                                                   MOTOR_DATA_CAN_NUM_TRANSMITTED, MOTOR_DATA_SVN_REVISION,
                                                                   MOTOR_DATA_F_P, MOTOR_DATA_I_D,
                                                                   MOTOR_DATA_IMAX_DEADBAND_SIGN};

    for(unsigned int i=0; i<nb_motor_data; ++i)
    {
      ConfPair tmp;
      tmp.first = base_topic + topics[i];
      tmp.second = data_types[i];
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
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
