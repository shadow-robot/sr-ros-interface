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
#include <pr2_controllers_msgs/JointControllerState.h>

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
    eoSRAutomaticPidTuning(const eoSRAutomaticPidTuning &)
    {
    // START Code of copy Ctor of an eoSRAutomaticPidTuning object
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
    return last_msg.error;
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
  void set_pid_service(std::string topic)
  {

  }

  std::vector<int> get_pid_settings() const
  {
    return pid_settings;
  }
  void set_pid_settings(std::vector<int> pids)
  {
    pid_settings = pids;
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

  pr2_controllers_msgs::JointControllerState last_msg;
  // END   Private data of an eoSRAutomaticPidTuning object
};

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
