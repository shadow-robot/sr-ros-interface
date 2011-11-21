/** -*- mode: c++; c-indent-level: 4; c++-member-init-indent: 8; comment-column: 35; -*-

The above line is usefulin Emacs-like editors
 */

/*
Template for EO objects initialization in EO
============================================
*/

#ifndef _eoSRAutomaticPidTuningInit_h
#define _eoSRAutomaticPidTuningInit_h

// include the base definition of eoInit
#include <eoInit.h>
#include <sr_utilities/sr_math_utils.hpp>
#include "sr_movements/movement_publisher.hpp"
#include <boost/algorithm/string.hpp>
/**
 *  Always write a comment in this format before class definition
 *  if you want the class to be documented by Doxygen
 *
 * There is NO ASSUMPTION on the class GenoypeT.
 * In particular, it does not need to derive from EO (e.g. to initialize
 *    atoms of an eoVector you will need an eoInit<AtomType>)
 */
template <class GenotypeT>
class eoSRAutomaticPidTuningInit: public eoInit<GenotypeT> {
public:
  /// Ctor - no requirement
// START eventually add or modify the anyVariable argument
  eoSRAutomaticPidTuningInit(std::vector<int> seed, std::vector<int> max_variations, std::string joint_name, shadowrobot::MovementPublisher mvt_pub, int controller_type)
  //  eoSRAutomaticPidTuningInit( varType  _anyVariable) : anyVariable(_anyVariable)
  // END eventually add or modify the anyVariable argument
  {
    // START Code of Ctor of an eoSRAutomaticPidTuningInit object
    this->controller_type = controller_type;

    for ( unsigned int i=0; i < seed.size(); ++i)
    {
      int min_value = seed[i] - max_variations[i];
      if( min_value < 0 )
        min_value = 0;
      int max_value = seed[i] + max_variations[i];

      min_range.push_back(min_value);
      max_range.push_back(max_value);
    }

    switch( controller_type )
    {
    case GAZEBO_CONTROLLER:
      //generate the controller state topic, the controller command topic
      // and the service name to change the pids.
      state_topic = "/"+joint_name + "_controller/state";
      command_topic = "/"+joint_name + "_controller/command";

      pid_service = "/"+joint_name + "_controller/set_gains";
      break;

    case MOTOR_CONTROLLER:
      state_topic = "/sh_"+joint_name+"_effort_controller/state";
      command_topic = "/sh_"+joint_name+"_effort_controller/command";

      pid_service = "/realtime_loop/change_force_PID_"+ boost::to_upper_copy(joint_name) ;
      break;

    case POSITION_CONTROLLER:
      state_topic = "/sh_"+joint_name+"_position_controller/state";
      command_topic = "/sh_"+joint_name+"_position_controller/command";

      pid_service = "/sh_"+joint_name+"_position_controller/set_gains";
      break;
    case VELOCITY_CONTROLLER:
      std::cout << " Automatic tuning for Velocity controllers not implemented yet" << std::endl;
      break;
    case MIXED_CONTROLLER:
      std::cout << " Automatic tuning for Mixed controllers not implemented yet" << std::endl;
      break;
    }

    mvt_publisher = mvt_pub;

    // END   Code of Ctor of an eoSRAutomaticPidTuningInit object
  }


  /** initialize a genotype
   *
   * @param _genotype  generally a genotype that has been default-constructed
   *                   whatever it contains will be lost
   */
  void operator()(GenotypeT & _genotype)
  {
    // START Code of random initialization of an eoSRAutomaticPidTuning object
    // END   Code of random initialization of an eoSRAutomaticPidTuning object
    _genotype.invalidate();	   // IMPORTANT in case the _genotype is old

    std::vector<int> initial_pids;
    for ( unsigned int i=0; i < min_range.size(); ++i)
    {
      int min_value = min_range[i];
      int max_value = max_range[i];
      int gene = sr_math_utils::Random::instance().generate<int>(min_value, max_value);

      initial_pids.push_back( gene );
    }

    _genotype.set_state_topic( state_topic );
    _genotype.set_command_topic( command_topic );
    _genotype.set_mvt_publisher( mvt_publisher );
    _genotype.set_pid_service( pid_service, controller_type );

    _genotype.set_pid_settings(initial_pids);
    _genotype.set_min_range(min_range);
    _genotype.set_max_range(max_range);
  }

private:
// START Private data of an eoSRAutomaticPidTuningInit object
  std::vector<int> min_range;
  std::vector<int> max_range;

  std::string state_topic;
  std::string command_topic;
  std::string pid_service;

  int controller_type;

  shadowrobot::MovementPublisher mvt_publisher;
// END   Private data of an eoSRAutomaticPidTuningInit object
};

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
