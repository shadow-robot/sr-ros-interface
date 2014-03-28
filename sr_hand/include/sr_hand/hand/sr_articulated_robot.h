/**
 * @file   sr_articulated_robot.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue May 25 17:50:47 2010
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
 * @brief  This is a parent class for the different types of Shadowhand we can
 * have. It makes it possible to swap from a virtual to a real hand while using
 * the same utilities to interact with the hand.
 * One (or more) ROS subscriber and publisher can then share the same instance
 * of a Shadowhand object to update the information contained in this object.
 *
 */

#ifndef   	SHADOWHAND_H_
# define   	SHADOWHAND_H_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

//self_test
#include "diagnostic_msgs/SelfTest.h"
#include "self_test/self_test.h"

#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>
#include <boost/assign.hpp>

#ifdef GAZEBO
#include <sensor_msgs/JointState.h>
#endif


namespace debug_values
{
  ///a map containing the names and offsets of the smart motor node
  static const std::map<const std::string, const unsigned int> names_and_offsets
    = boost::assign::map_list_of ("Sensor PID last in",         0) \
                                 ("Sensor PID iState",          1) \
                                 ("Sensor PID last D",          2) \
                                 ("Sensor PID last out",        3) \
                                 ("PID last in",                4) \
                                 ("PID iState",                 5) \
                                 ("PID last D",                 6) \
                                 ("PID last out",               7) \
                                 ("Strain Gauge Offset 0",      8) \
                                 ("Strain Gauge Offset 1",      9) \
                                 ("Num setup Msgs received",   10) \
                                 ("Num sensor Msgs received",  11) \
                                 ("Sensor Val (motor set P)",  12) \
                                 ("Sensor Val (motor sensor)", 13) \
                                 ("H-Bridge Duty",             14) \
                                 ("Duty Temp",                 15) ;
}


namespace shadowrobot
{
/**
 * This struct contains all the information regarding each joints.
 */
  struct JointData
  {
    double position;
    double target;
    double temperature;
    double current;
    double force;
    std::string flags;
    int jointIndex;
    double min;
    double max;
    short isJointZero;

    /**
     * GAZEBO and the etherCAT wrapper have one publisher / subscriber
     * per joint. We store those in the JointData struct to be able
     * to get and send data to the Gazebo model and the EherCAT hand
     * with our standard ROS interface.
     */
    int publisher_index;
    ros::Time last_pos_time;
    double last_pos;
    double velocity;

  JointData() :
    position(0.0), target(0.0), temperature(0.0), current(0.0), force(0.0), flags(""), jointIndex(0), min(0.0), max(90.0), isJointZero(0), publisher_index(0), last_pos_time(0.0), last_pos(0.0), velocity(0.0)
    {
    }

  JointData(JointData& jd) :
    position(jd.position), target(jd.target), temperature(jd.temperature), current(jd.current), force(jd.force),
      flags(jd.flags), jointIndex(jd.jointIndex), min(jd.min), max(jd.max), isJointZero(jd.isJointZero), publisher_index(jd.publisher_index), last_pos_time(jd.last_pos_time), last_pos(jd.last_pos), velocity(jd.velocity)
    {
    }

  JointData(const JointData& jd) :
    position(jd.position), target(jd.target), temperature(jd.temperature), current(jd.current), force(jd.force),
      flags(jd.flags), jointIndex(jd.jointIndex), min(jd.min), max(jd.max), isJointZero(jd.isJointZero), publisher_index(jd.publisher_index), last_pos_time(jd.last_pos_time), last_pos(jd.last_pos), velocity(jd.velocity)
    {
    }
  };

/**
 * An enum containing all the different types of parameters for the controllers.
 */
  enum controller_parameters
  {
    PARAM_sensor, //!<select the sensor that is being controller
    PARAM_target, //!< select the sensor to read the setpoint from
    PARAM_valve, //!< select which valve is being controlled, if appropriate
    PARAM_deadband, //!< select the deadband value, if appropriate (actual deadband used is \f$2^{(deadband-16)}\f$)
    PARAM_p, //!< change the proportional gain
    PARAM_i, //!< change the integral gain
    PARAM_d, //!< change the derivative gain
    PARAM_imax, //!< PARAM_imax
    PARAM_output_offset, //!< PARAM_output_offset
    PARAM_shift, //!< PARAM_shift
    PARAM_output_max, //!< PARAM_output_max

    PARAM_motor_maxforce, //!< PARAM_motor_maxforce
    PARAM_motor_safeforce, //!< PARAM_motor_safeforce
    PARAM_force_p, //!< PARAM_force_p
    PARAM_force_i, //!< PARAM_force_i
    PARAM_force_d, //!< PARAM_force_d
    PARAM_force_imax, //!< PARAM_force_imax
    PARAM_force_out_shift, //!< PARAM_force_out_shift
    PARAM_force_deadband, //!< PARAM_force_deadband
    PARAM_force_offset, //!< PARAM_force_offset
    PARAM_sensor_imax, //!< PARAM_sensor_imax
    PARAM_sensor_out_shift,//!< PARAM_sensor_out_shift
    PARAM_sensor_deadband, //!< PARAM_sensor_deadband
    PARAM_sensor_offset, //!< PARAM_sensor_offset
    PARAM_max_temperature, //!< PARAM_max_temperature
    PARAM_max_current, //!< PARAM_max_current
    PARAM_type_of_sensor, //!< PARAM_type_of_sensor
    PARAM_type_of_setpoint
//!< PARAM_type_of_setpoint

  };

/**
 * Description of a parameter for a controller.
 */
  struct Parameters
  {
    ///name of the parameter
    std::string name;
    ///value of the parameter
    std::string value;

  Parameters() :
    name(""), value("")
    {
    }

  Parameters( Parameters& param ) :
    name(param.name), value(param.value)
    {
    }

  Parameters( const Parameters& param ) :
    name(param.name), value(param.value)
    {
    }
  };

/**
 * A vector containing all the Parameters for a given controller.
 */
  struct JointControllerData
  {
    std::vector<Parameters> data;

  JointControllerData() :
    data()
    {
    }

  JointControllerData( JointControllerData& jcd ) :
    data(jcd.data)
    {
    }

  JointControllerData( const JointControllerData& jcd ) :
    data(jcd.data)
    {
    }
  };



/**
 * The information being published by the Diagnostic publisher
 */
  struct DiagnosticData
  {
    /// the name of the joint
    std::string joint_name;
    /// the level of alert: 0 = OK, 1 = WARNING, 2 = ERROR
    short level;

    /// a string containing flags: FORCE, TEMPERATURE, ... indicating the different cutouts / warning sent by the hand.
    std::string flags;

    ///the channel number of the target sensor
    int target_sensor_num;
    ///the channel number of the position sensor
    int position_sensor_num;
    ///the actual value of the target
    double target;

    ///the debug values
    std::map<std::string, int> debug_values;
    ///the actual value of the position
    double position;

    ///temperature value
    double temperature;
    ///current value
    double current;
    ///force value
    double force;

    //values read from the debug node.
    uint64_t num_sensor_msgs_received;

  };

/**
 * @brief  This is a parent class for the different types of Shadowhand we can
 * have. It makes it possible to swap from a virtual to a real hand while using
 * the same utilities to interact with the hand.
 * One (or more) ROS subscriber and publisher can then share the same instance
 * of a Shadowhand object to update the information contained in this object.
 */
  class SRArticulatedRobot : boost::noncopyable
  {
  public:
    /**
     * empty constructor.
     */
    SRArticulatedRobot()
    {
    }
    ;

    /**
     * empty destructor.
     */
    virtual ~SRArticulatedRobot()
    {
    }
    ;

    ///@see joints_map
    typedef std::map<std::string, JointData> JointsMap;
    ///@see parameters_map
    typedef std::map<std::string, enum controller_parameters> ParametersMap;

    /**
     * Generic method called to pass a new target to an articulated robot.
     * @param joint_name The Joint in joints_map you wish to send the target to.
     * @param target The target in degree
     * @return 0 if success ; -1 if error
     */
    virtual short sendupdate( std::string joint_name, double target ) = 0;

    /**
     * Get the joint data for a specific joint. @see JointData
     * @param joint_name The name of the joint, as specified in joints_map.
     * @return The information regarding this joint.
     */
    virtual JointData getJointData( std::string joint_name ) = 0;

    /**
     * Get the data for all the joints.
     * @return a mapping between the joints names and the information for each joint.
     */
    virtual JointsMap getAllJointsData() = 0;

    /**
     * Set the controller parameters for a given controller name.
     * @param contrlr_name The name of the controller to setup.
     * @param ctrlr_data The data to pass to this controller.
     * @return 0 if success.
     */
    virtual short setContrl( std::string contrlr_name, JointControllerData ctrlr_data ) = 0;

    /**
     * Get the controller parameters for a given controller name.
     * @param contrlr_name the name of the controller.
     * @return The parameters of this controller
     */
    virtual JointControllerData getContrl( std::string contrlr_name ) = 0;

    /**
     * Set the config of the palm
     * @todo Not implemented yet
     *
     * @param myConfig
     * @return
     */
    virtual short setConfig( std::vector<std::string> myConfig ) = 0;

    /**
     * Get the config of the palm
     * @todo Not implemented yet
     *
     * @param joint_name
     */
    virtual void getConfig( std::string joint_name ) = 0;

    /**
     * Get the diagnostics for the whole articulated robot.
     * @return A vector containing all the diagnostics for the robot (motor information, etc...)
     */
    virtual std::vector<DiagnosticData> getDiagnostics() = 0;

    /// A mapping between the joint names and the information regarding those joints.
    JointsMap joints_map;

    /// A mapping between the parameter names and their values.
    ParametersMap parameters_map;

    boost::mutex joints_map_mutex;
    boost::mutex parameters_map_mutex;
    boost::mutex controllers_map_mutex;

  protected:
    ///this is the handle for the self tests.
    boost::shared_ptr<self_test::TestRunner> self_test;
#ifdef GAZEBO
    std::vector<ros::Publisher> gazebo_publishers;
    ros::Subscriber gazebo_subscriber;
#endif
  }; //end class

}//end namespace


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif 	    /* !SHADOWHAND_H_ */
