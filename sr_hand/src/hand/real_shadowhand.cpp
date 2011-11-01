/**
 * @file   real_shadowhand.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue May 25 17:50:42 2010
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
 * @brief
 *
 *
 */

#include <ros/ros.h>

#include "sr_hand/hand/real_shadowhand.h"
//our robot code
#include <robot/config.h>
#include <robot/robot.h>
#include <robot/hand_protocol.h>
#include <robot/hand.h>
#ifdef FINGER
#define LAST_HAND_JOINT NUM_FINGER_JOINTS
#else
#define LAST_HAND_JOINT START_OF_ARM
#endif

namespace shadowrobot
{
  RealShadowhand::RealShadowhand() :
    SRArticulatedRobot()
  {
    /* We need to attach the program to the robot, or fail if we cannot. */
    if( robot_init() < 0 )
    {
      ROS_FATAL("Robot interface broken\n");
      ROS_BREAK();
    }

    /* We need to attach the program to the hand as well, or fail if we cannot. */
    if( hand_init() < 0 )
    {
      ROS_FATAL("Hand interface broken\n");
      ROS_BREAK();
    }

    initializeMap();

    self_test = boost::shared_ptr<self_test::TestRunner>(new self_test::TestRunner());

    self_test->add("Pretest", this, &RealShadowhand::pretest);
    self_test->add("Number of messages Received", this, &RealShadowhand::test_messages);
    self_test->add("Posttest", this, &RealShadowhand::posttest);
  }

  RealShadowhand::~RealShadowhand()
  {
  }

  void RealShadowhand::initializeMap()
  {
    joints_map_mutex.lock();
    parameters_map_mutex.lock();

    JointData tmpData;

    tmpData.position = 0.0;
    tmpData.target = 0.0;
    tmpData.temperature = 0.0;
    tmpData.current = 0.0;
    tmpData.force = 0.0;
    tmpData.flags = "";

    for( unsigned int i = 0; i < LAST_HAND_JOINT; i++ )
    {
      std::string name = hand_joints[i].joint_name;
      tmpData.jointIndex = i;

      joints_map[name] = tmpData;

      ROS_DEBUG("NAME[%d]: %s ", i, name.c_str());
    }

    parameters_map["d"] = PARAM_d;
    parameters_map["i"] = PARAM_i;
    parameters_map["p"] = PARAM_p;
    parameters_map["target"] = PARAM_target;
    parameters_map["sensor"] = PARAM_sensor;

    parameters_map["valve"] = PARAM_valve;
    parameters_map["dead"] = PARAM_deadband;
    parameters_map["deadband"] = PARAM_deadband;
    parameters_map["imax"] = PARAM_imax;
    parameters_map["offset"] = PARAM_output_offset;
    parameters_map["shift"] = PARAM_shift;
    parameters_map["max"] = PARAM_output_max;

    //! the parameters for the motors
    parameters_map["motor_maxforce"] = PARAM_motor_maxforce;
    parameters_map["motor_safeforce"] = PARAM_motor_safeforce;

    parameters_map["force_p"] = PARAM_force_p;
    parameters_map["force_i"] = PARAM_force_i;
    parameters_map["force_d"] = PARAM_force_d;

    parameters_map["force_imax"] = PARAM_force_imax;
    parameters_map["force_out_shift"] = PARAM_force_out_shift;
    parameters_map["force_deadband"] = PARAM_force_deadband;
    parameters_map["force_offset"] = PARAM_force_offset;

    parameters_map["sensor_imax"] = PARAM_sensor_imax;
    parameters_map["sensor_out_shift"] = PARAM_sensor_out_shift;
    parameters_map["sensor_deadband"] = PARAM_sensor_deadband;
    parameters_map["sensor_offset"] = PARAM_sensor_offset;
    parameters_map["max_temp"] = PARAM_max_temperature;
    parameters_map["max_temperature"] = PARAM_max_temperature;
    parameters_map["max_current"] = PARAM_max_current;

    parameters_map_mutex.unlock();
    joints_map_mutex.unlock();
  }

  short RealShadowhand::sendupdate( std::string joint_name, double target )
  {
    joints_map_mutex.lock();

    //search the sensor in the hash_map
    JointsMap::iterator iter = joints_map.find(joint_name);

    if( iter != joints_map.end() )
    {
      JointData tmpData = joints_map.find(joint_name)->second;
      int index_hand_joints = tmpData.jointIndex;

      //trim to the correct range
      if( target < hand_joints[index_hand_joints].min_angle )
        target = hand_joints[index_hand_joints].min_angle;
      if( target > hand_joints[index_hand_joints].max_angle )
        target = hand_joints[index_hand_joints].max_angle;

      //here we update the actual hand angles
      robot_update_sensor(&hand_joints[index_hand_joints].joint_target, target);
      joints_map_mutex.unlock();
      return 0;
    }

    ROS_DEBUG("Joint %s not found", joint_name.c_str());

    joints_map_mutex.unlock();
    return -1;
  }

  JointData RealShadowhand::getJointData( std::string joint_name )
  {
    joints_map_mutex.lock();

    JointsMap::iterator iter = joints_map.find(joint_name);

    //joint not found
    if( iter == joints_map.end() )
    {
      ROS_ERROR("Joint %s not found.", joint_name.c_str());
      JointData noData;
      noData.position = 0.0;
      noData.target = 0.0;
      noData.temperature = 0.0;
      noData.current = 0.0;
      noData.force = 0.0;
      noData.flags = "";
      noData.jointIndex = 0;

      joints_map_mutex.unlock();
      return noData;
    }

    //joint found
    JointData tmpData = iter->second;
    int index = tmpData.jointIndex;

    tmpData.position = robot_read_sensor(&hand_joints[index].position);
    tmpData.target = robot_read_sensor(&hand_joints[index].joint_target);

    //more information
    if( hand_joints[index].a.smartmotor.has_sensors )
    {
      tmpData.temperature = robot_read_sensor(&hand_joints[index].a.smartmotor.temperature);
      tmpData.current = robot_read_sensor(&hand_joints[index].a.smartmotor.current);
      tmpData.force = robot_read_sensor(&hand_joints[index].a.smartmotor.torque);
      tmpData.flags = "";
    }

    joints_map[joint_name] = tmpData;

    joints_map_mutex.unlock();
    return tmpData;
  }

  SRArticulatedRobot::JointsMap RealShadowhand::getAllJointsData()
  {
    //update the map for each joints
    for( JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it )
      getJointData(it->first);

    //hack for C6M2 with 4 fingers with coupling J1+J2=J0
    //copy J0/2 to J1 and J2 for FF MF RF LF
    joints_map_mutex.lock();
    
    std::string fingername("FF");
    
    JointsMap::iterator it_j0 = joints_map.find(fingername+"J0");
    JointsMap::iterator it_j1 = joints_map.find(fingername+"J1");
    JointsMap::iterator it_j2 = joints_map.find(fingername+"J2");
    it_j1->second.target = it_j0->second.target/2.0;
    it_j2->second.target = it_j0->second.target/2.0;

    fingername.assign("MF");
    it_j0 = joints_map.find(fingername+"J0");
    it_j1 = joints_map.find(fingername+"J1");
    it_j2 = joints_map.find(fingername+"J2");
    it_j1->second.target = it_j0->second.target/2.0;
    it_j2->second.target = it_j0->second.target/2.0;

    fingername.assign("RF");
    it_j0 = joints_map.find(fingername+"J0");
    it_j1 = joints_map.find(fingername+"J1");
    it_j2 = joints_map.find(fingername+"J2");
    it_j1->second.target = it_j0->second.target/2.0;
    it_j2->second.target = it_j0->second.target/2.0;

    fingername.assign("LF");
    it_j0 = joints_map.find(fingername+"J0");
    it_j1 = joints_map.find(fingername+"J1");
    it_j2 = joints_map.find(fingername+"J2");
    it_j1->second.target = it_j0->second.target/2.0;
    it_j2->second.target = it_j0->second.target/2.0;
    
    joints_map_mutex.unlock();

    JointsMap tmp = JointsMap(joints_map);

    //return the map
    return tmp;
  }

  short RealShadowhand::setContrl( std::string contrlr_name, JointControllerData ctrlr_data )
  {
    parameters_map_mutex.lock();

    struct controller_config myConfig;
    memset(&myConfig, 0, sizeof(myConfig));

    //set the nodename from contrlr_name
    myConfig.nodename = contrlr_name.c_str();

    controller_read_from_hardware(&myConfig);
    ROS_DEBUG("%s", controller_to_string(&myConfig));

    for( unsigned int i = 0; i < ctrlr_data.data.size(); ++i )
    {
      std::string name = ctrlr_data.data[i].name;
      ParametersMap::iterator iter = parameters_map.find(name);

      //parameter not found
      if( iter == parameters_map.end() )
      {
        ROS_ERROR("Parameter %s not found.", name.c_str());
        continue;
      }

      //parameter found
      controller_update_param(&myConfig, (controller_param)iter->second, ctrlr_data.data[i].value.c_str());
    }

    parameters_map_mutex.unlock();

    int result_ctrlr = controller_write_to_hardware(&myConfig);
    if( result_ctrlr )
    {
      ROS_ERROR("Failed to update contrlr (%s)", myConfig.nodename );
      return -1;
    }

    return 0;
  }

  JointControllerData RealShadowhand::getContrl( std::string contrlr_name )
  {
    struct controller_config myConfig;
    memset(&myConfig, 0, sizeof(myConfig));

    //set the nodename from contrlr_name
    myConfig.nodename = contrlr_name.c_str();

    controller_read_from_hardware(&myConfig);

    JointControllerData tmp_data;

    ROS_WARN("The get contrlr function is not implemented in the real shadowhand.");

    return tmp_data;

  }

  short RealShadowhand::setConfig( std::vector<std::string> myConfig )
  {
    ROS_WARN("The set config function is not implemented in the real shadowhand.");

    /*
      hand_protocol_config_t cfg;
      hand_protocol_get_config(cfg);

      //set the transmit rate value
      int value = msg->rate_value;
      cfg->u.palm.tx_freq[num]=value;

      //send the config to the palm.
      hand_protocol_set_config(cfg);
    */

    return 0;
  }

  void RealShadowhand::getConfig( std::string joint_name )
  {
    ROS_WARN("The get config function is not yet implement in the real shadow hand.");
  }

  std::vector<DiagnosticData> RealShadowhand::getDiagnostics()
  {
    //it's alright to do the tests when we're publishing the diagnostics
    self_test->checkTest();

    std::vector<DiagnosticData> returnVector;

    DiagnosticData tmpData;
    std::stringstream ss;

    //get the data from the hand
    for( unsigned int index = 0; index < START_OF_ARM; ++index )
    {
      tmpData.joint_name = std::string(hand_joints[index].joint_name);
      tmpData.level = 0;

      tmpData.position = robot_read_sensor(&hand_joints[index].position);
      tmpData.target = robot_read_sensor(&hand_joints[index].joint_target);

      //more information
      if( hand_joints[index].a.smartmotor.has_sensors )
      {
        //reads the number of received sensor msgs from the debug node.
        int res;
        if( *(&hand_joints[index].a.smartmotor.debug_nodename) != NULL)
        {
          std::string debug_node_name = *(&hand_joints[index].a.smartmotor.debug_nodename);

          //get all the debug values
          std::map<const std::string, const unsigned int>::const_iterator iter;
          for(iter = debug_values::names_and_offsets.begin();
              iter !=  debug_values::names_and_offsets.end(); ++iter)
          {
            struct sensor sensor_tmp;

            res = robot_channel_to_sensor(debug_node_name.c_str(), iter->second, &sensor_tmp);
            tmpData.debug_values[iter->first] = robot_read_incoming(&sensor_tmp);
          }
        }
	//	else
	//  ROS_ERROR_STREAM(tmpData.joint_name << ": no debug sensor" );
        //reads temperature current and force.
        tmpData.temperature = robot_read_sensor(&hand_joints[index].a.smartmotor.temperature);
        tmpData.current = robot_read_sensor(&hand_joints[index].a.smartmotor.current);
        tmpData.force = robot_read_sensor(&hand_joints[index].a.smartmotor.torque);

        //check for error_flags

        struct hand_protocol_flags fl;
        uint64_t uuid = robot_node_id(hand_joints[index].a.smartmotor.nodename);
        fl = hand_protocol_get_status_flags(uuid);
        if( fl.valid )
        {
          ss.clear();
          struct hand_protocol_flags_smart_motor f;
          f = fl.u.smart_motor;

          bool at_least_one_error_flag = false;
          if( f.nfault_pin )
          {
            at_least_one_error_flag = true;
            ss << "NFAULT ";
            ROS_WARN( "[%s]: NFAULT", hand_joints[index].joint_name );
          }
          if( f.temperature_cutout )
          {
            at_least_one_error_flag = true;
            ss << "TEMP ";
          }
          if( f.current_throttle )
          {
            at_least_one_error_flag = true;
            ss << "CURRENT ";
          }
          if( f.force_hard_limit )
          {
            at_least_one_error_flag = true;
            ss << "FORCE ";
          }
          if( hand_protocol_dead(uuid) )
          {
            at_least_one_error_flag = true;
            ss << "DEAD ";
          }

          //if a flag is up, then print a warning as well
          if( at_least_one_error_flag )
          {
            ROS_WARN( "[%s]: %s", hand_joints[index].joint_name, (ss.str()).c_str());
            tmpData.level = 1;
          }

          tmpData.flags = ss.str();
        }
      }

      returnVector.push_back(tmpData);
    }
    return returnVector;
  }

  ///////////////////
  //    TESTS      //
  ///////////////////
  void RealShadowhand::pretest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    ROS_INFO("Preparing the environment to run self tests.");

    //lock all the mutexes to make sure we're not publishing other messages
    joints_map_mutex.lock();
    parameters_map_mutex.lock();
    controllers_map_mutex.lock();

    //TODO: set the palm transmit rate to max?

    sleep(1);

    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Pretest completed successfully.");
  }

  void RealShadowhand::posttest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    ROS_INFO("Restoring the environment after the self tests.");

    //test finished, unlocking all the mutexes
    joints_map_mutex.unlock();
    parameters_map_mutex.unlock();
    controllers_map_mutex.unlock();

    //TODO: reset the palm transmit rate to previous state?

    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "Postest completed successfully.");
  }

  void RealShadowhand::test_messages(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    ROS_WARN("Starting the test: Number of messages Received");

    std::pair<unsigned char, std::string> test_result;
    test_result.first = diagnostic_msgs::DiagnosticStatus::OK;

    for(unsigned int index_freq=0; index_freq < sr_self_tests::msgs_frequency_size; ++index_freq)
    {
      ros::Rate test_rate(sr_self_tests::msgs_frequency[index_freq]);
      for(unsigned int index_joint=0; index_joint < sr_self_tests::joints_to_test_size; ++index_joint)
      {
        std::pair<unsigned char, std::string> tmp_test_result;
        tmp_test_result = test_messages_routine(sr_self_tests::joints_to_test[index_joint], sr_self_tests::nb_targets_to_send, test_rate);

        if( tmp_test_result.first == diagnostic_msgs::DiagnosticStatus::ERROR)
          test_result.first = diagnostic_msgs::DiagnosticStatus::ERROR;

        std::stringstream ss;
        ss << "\n[" << sr_self_tests::msgs_frequency[index_freq] << "Hz]: ";
        ss << tmp_test_result.second;

        test_result.second += ss.str();
      }
    }
    status.summary(test_result.first, test_result.second);
  }

  std::pair<unsigned char, std::string> RealShadowhand::test_messages_routine(std::string joint_name, unsigned int repeat, ros::Rate rate)
  {
    std::pair<unsigned char, std::string> test_result;

    //id should be motor board number
    std::string ID = "1";
    self_test->setID(ID.c_str());

    unsigned int nb_msgs_sent = 0;
    unsigned int nb_msgs_received = 0;

    //sending lots of data to one joint
    JointsMap::iterator iter_joints_map = joints_map.find(joint_name);

    struct sensor sensor_msgs_received;

    if( iter_joints_map == joints_map.end() )
    {
      std::stringstream ss;
      ss << "No messages sent: couldn't find joint "<<joint_name;

      test_result.first = diagnostic_msgs::DiagnosticStatus::ERROR;
      test_result.second = ss.str();
      return test_result;
    }

    //OK joint found
    JointData tmpData = joints_map.find(joint_name)->second;
    int index_hand_joints = tmpData.jointIndex;
    float target =  hand_joints[index_hand_joints].min_angle;

    //testing a joint which doesn't have a smartmotor
    if( !hand_joints[index_hand_joints].a.smartmotor.has_sensors )
    {
      std::stringstream ss;
      ss << "No messages sent: joint["<<joint_name<<"] doesn't have any motor attached";

      test_result.first = diagnostic_msgs::DiagnosticStatus::ERROR;
      test_result.second = ss.str();
      return test_result;
    }

    ROS_DEBUG("Checking the current number of received messages");

    int res;
    std::string debug_node_name = *(&hand_joints[index_hand_joints].a.smartmotor.debug_nodename);
    std::map<const std::string, const unsigned int>::const_iterator iter_debug_values =  debug_values::names_and_offsets.find("Num sensor Msgs received");

    res = robot_channel_to_sensor(debug_node_name.c_str(), iter_debug_values->second, &sensor_msgs_received);

    //check the number of messages already received when starting the test
    nb_msgs_received = robot_read_incoming(&sensor_msgs_received) - nb_msgs_received;

    sleep(1);

    //check if no other messages have been received
    if( nb_msgs_received != robot_read_incoming(&sensor_msgs_received))
    {
      std::stringstream ss;
      ss <<  "New messages were received on the joint[" <<  joint_name.c_str() << "]." ;
      test_result.first = diagnostic_msgs::DiagnosticStatus::ERROR;
      test_result.second = ss.str();

      return test_result;
    }
    //ok still the same number of messages

    ROS_DEBUG("Sending lots of messages.");

    for(; nb_msgs_sent < repeat; ++nb_msgs_sent)
    {
      //send values to the sensor
      robot_update_sensor(&hand_joints[index_hand_joints].joint_target, target);
      rate.sleep();
      ROS_DEBUG_STREAM("msg "<< nb_msgs_sent<< "/"<<sr_self_tests::nb_targets_to_send);
    }

    ROS_DEBUG("Done sending the messages.");
    //wait for all the messages to be received?
    sleep(0.5);

    ROS_DEBUG("Reading the number of received messages.");
    //compute the number of messages received during the test
    nb_msgs_received = robot_read_incoming(&sensor_msgs_received) - nb_msgs_received;

    if( nb_msgs_sent == nb_msgs_received)
    {
      std::stringstream ss;
      ss <<  nb_msgs_sent << " messages sent, all received";
      test_result.first = diagnostic_msgs::DiagnosticStatus::OK;
      test_result.second = ss.str();
      return test_result;
    }
    else
    {
      std::stringstream ss;
      ss << nb_msgs_sent << " messages sent, "<<nb_msgs_received << " messages received";
      test_result.first = diagnostic_msgs::DiagnosticStatus::ERROR;
      test_result.second = ss.str();
      return test_result;
    }
  }
} //end namespace

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
