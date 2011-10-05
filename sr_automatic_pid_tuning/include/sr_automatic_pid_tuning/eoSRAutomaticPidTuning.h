/** -*- mode: c++; c-indent-level: 4; c++-member-init-indent: 8; comment-column: 35; -*-

The above line is usefulin Emacs-like editors
 */

/*
  Template for creating a new representation in EO
  ================================================

  Mandatory:
  - a default constructor (constructor without any argument)
  - the I/O functions (readFrom and printOn)

  However, if you are using dynamic memory, there are 2 places
  to allocate it: the default constructor (if possible?), or, more in
  the EO spirit, the eoInit object, that you will need to write anyway
  (template file init.tmpl).

  But remember that a COPY CONSTRUCTOR will be used in many places in EO,
  so make sure that the default copy constructor works, or, even better,
  do write your own if in doubt.
  And of course write the corresponding destructor!

*/

#ifndef _eoSRAutomaticPidTuning_h
#define _eoSRAutomaticPidTuning_h

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sr_robot_msgs/SetPidGains.h>
#include <sr_robot_msgs/ForceController.h>
#include <pr2_controllers_msgs/JointControllerState.h>

static enum {
  MOTOR_CONTROLLER,
  POSITION_CONTROLLER,
  VELOCITY_CONTROLLER,
  MIXED_CONTROLLER,
  GAZEBO_CONTROLLER
} controller_types;

/**
 *  Always write a comment in this format before class definition
 *  if you want the class to be documented by Doxygen

 * Note that you MUST derive your structure from EO<fitT>
 * but you MAY use some other already prepared class in the hierarchy
 * like eoVector for instance, if you handle a vector of something....

 * If you create a structure from scratch,
 * the only thing you need to provide are
 *        a default constructor
 *        IO routines printOn and readFrom
 *
 * Note that operator<< and operator>> are defined at EO level
 * using these routines
 */
template< class FitT>
class eoSRAutomaticPidTuning: public EO<FitT> {
public:
  /** Ctor: you MUST provide a default ctor.
   * though such individuals will generally be processed
   * by some eoInit object
   */
  eoSRAutomaticPidTuning()
  {
    // START Code of default Ctor of an eoSRAutomaticPidTuning object
    // END   Code of default Ctor of an eoSRAutomaticPidTuning object
  }

  /** Copy Ctor: you MUST provide a copy ctor if the default
   * one is not what you want
   * If this is the case, uncomment and fill the following
   */
  /*
    eoSRAutomaticPidTuning(const eoSRAutomaticPidTuning & eo_pid_tuning)
    {
    // START Code of copy Ctor of an eoSRAutomaticPidTuning object
    pid_settings = eo_pid_tuning.pid_settings;
    min_range = eo_pid_tuning.min_range;
    max_range = eo_pid_tuning.max_range;

    nh = eo_pid_tuning.nh;
    pub = eo_pid_tuning.pub;
    sub = eo_pid_tuning.sub;
    pid_service = eo_pid_tuning.pid_service;

    controller_type = eo_pid_tuning.controller_type;
    mvt_publisher = eo_pid_tuning.mvt_publisher;
    last_msg = eo_pid_tuning.last_msg;

    try
    {
    this->fitness( eo_pid_tuning.fitness() );
    }
    catch(exception& e)
    {
    this->fitness( 1000000000.0 );
    }
    // END   Code of copy Ctor of an eoSRAutomaticPidTuning object
    }
  */

  virtual ~eoSRAutomaticPidTuning()
  {
    // START Code of Destructor of an eoEASEAGenome object
    // END   Code of Destructor of an eoEASEAGenome object
  }

  virtual string className() const { return "eoSRAutomaticPidTuning"; }

  /** printing... */
  void printOn(ostream& os) const
  {
    // First write the fitness
    EO<FitT>::printOn(os);
    os << ' ';
    // START Code of default output
    /** HINTS
     * in EO we systematically write the sizes of things before the things
     * so readFrom is easier to code (see below)
     */

    os << "size: " << pid_settings.size() << " values: ";
    for( unsigned int i=0 ; i < pid_settings.size(); ++i)
    {
      os << pid_settings[i] << " ";
    }
    // END   Code of default output
  }

  /** reading...
   * of course, your readFrom must be able to read what printOn writes!!!
   */
  void readFrom(istream& is)
  {
    // of course you should read the fitness first!
    EO<FitT>::readFrom(is);
    // START Code of input

    ROS_ERROR_STREAM("readfrom: " << is);
    /** HINTS
     * remember the eoSRAutomaticPidTuning object will come from the default ctor
     * this is why having the sizes written out is useful
     */

    // END   Code of input
  }

  void controller_state_callback(const pr2_controllers_msgs::JointControllerState& msg)
  {
    last_msg = msg;
  }

  double get_last_error()
  {
    double error = last_msg.error;
    return error;
  }

  void publish(double target)
  {
    std_msgs::Float64 msg;
    msg.data = target;
    pub.publish( msg );
  }

  /////
  // Set the ROS subscriber / publisher / service proxy.
  void set_state_topic(std::string topic)
  {
    sub = nh.subscribe(topic, 2, &eoSRAutomaticPidTuning::controller_state_callback, this);
  }
  void set_command_topic(std::string topic)
  {
    pub = nh.advertise<std_msgs::Float64>(topic, 5);
  }
  void set_pid_service(std::string topic, int controller_type)
  {
    this->controller_type = controller_type;

    switch( controller_type )
    {
    case GAZEBO_CONTROLLER:
    case POSITION_CONTROLLER:
      pid_service = nh.serviceClient<sr_robot_msgs::SetPidGains>(topic);
      break;
    case MOTOR_CONTROLLER:
      pid_service = nh.serviceClient<sr_robot_msgs::ForceController>(topic);
      break;

    case VELOCITY_CONTROLLER:
      std::cout << " Automatic tuning for Velocity controllers not implemented yet" << std::endl;
      break;
    case MIXED_CONTROLLER:
      std::cout << " Automatic tuning for Mixed controllers not implemented yet" << std::endl;
      break;
    }
  }

  std::vector<int> get_pid_settings() const
  {
    return pid_settings;
  }
  void set_pid_settings(std::vector<int> pids)
  {
    pid_settings = pids;

    switch( controller_type )
    {
    case GAZEBO_CONTROLLER:
    case POSITION_CONTROLLER:
    {
      sr_robot_msgs::SetPidGains::Request gazebo_pid_req;
      sr_robot_msgs::SetPidGains::Response gazebo_pid_res;

      gazebo_pid_req.p = -pids[0];
      gazebo_pid_req.i = -pids[1];
      gazebo_pid_req.d = 0.0;
      gazebo_pid_req.i_clamp = pids[2];
      gazebo_pid_req.max_force = 1023;
      gazebo_pid_req.deadband = 0.0;
      gazebo_pid_req.friction_deadband = 5000;

      if (!pid_service.call(gazebo_pid_req, gazebo_pid_res))
      {
        ROS_ERROR("failed to set pid");
      }
    }
    break;

    case MOTOR_CONTROLLER:
    {
      sr_robot_msgs::ForceController::Request motor_pid_req;
      sr_robot_msgs::ForceController::Response motor_pid_res;

      motor_pid_req.f = pids[0];
      motor_pid_req.p = pids[1];
      motor_pid_req.i = pids[2];
      motor_pid_req.d = pids[3];
      motor_pid_req.imax = pids[4];
      motor_pid_req.maxpwm = 1023;
      motor_pid_req.deadband = 0;
      motor_pid_req.sgleftref = 0;
      motor_pid_req.sgrightref = 0;
      motor_pid_req.sign = 0;

      if (!pid_service.call(motor_pid_req, motor_pid_res))
      {
        ROS_ERROR("failed to set pid");
      }
    }
    break;

    case VELOCITY_CONTROLLER:
      std::cout << " Automatic tuning for Velocity controllers not implemented yet" << std::endl;
      break;
    case MIXED_CONTROLLER:
      std::cout << " Automatic tuning for Mixed controllers not implemented yet" << std::endl;
      break;
    }
  }

  void set_mvt_publisher(shadowrobot::MovementPublisher mvt_pub)
  {
    mvt_publisher = mvt_pub;
    mvt_pub.set_publisher( pub );
  }
  void execute_mvt_step(int index_step)
  {
    mvt_publisher.execute_step(index_step, 0);
  }

  std::vector<int> get_min_range() const
  {
    return min_range;
  }
  void set_min_range(std::vector<int> mins)
  {
    min_range = mins;
  }

  std::vector<int> get_max_range() const
  {
    return max_range;
  }
  void set_max_range(std::vector<int> maxs)
  {
    max_range = maxs;
  }

private:			   // put all data here
  // START Private data of an eoSRAutomaticPidTuning object
  std::vector<int> pid_settings;
  std::vector<int> min_range;
  std::vector<int> max_range;

  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::ServiceClient pid_service;

  int controller_type;

  shadowrobot::MovementPublisher mvt_publisher;

  pr2_controllers_msgs::JointControllerState last_msg;
  // END   Private data of an eoSRAutomaticPidTuning object
};

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
