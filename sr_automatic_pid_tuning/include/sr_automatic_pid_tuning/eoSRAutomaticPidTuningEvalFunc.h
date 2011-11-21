/*
Template for evaluator in EO, a functor that computes the fitness of an EO
==========================================================================
*/

#ifndef _eoSRAutomaticPidTuningEvalFunc_h
#define _eoSRAutomaticPidTuningEvalFunc_h

// include whatever general include you need
#include <stdexcept>
#include <fstream>

// include the base definition of eoEvalFunc
#include "eoEvalFunc.h"

/**
  Always write a comment in this format before class definition
  if you want the class to be documented by Doxygen
*/
template <class EOT>
class eoSRAutomaticPidTuningEvalFunc : public eoEvalFunc<EOT>
{
public:
  /// Ctor - no requirement
// START eventually add or modify the anyVariable argument
  eoSRAutomaticPidTuningEvalFunc()
  //  eoSRAutomaticPidTuningEvalFunc( varType  _anyVariable) : anyVariable(_anyVariable)
  // END eventually add or modify the anyVariable argument
  {
    // START Code of Ctor of an eoSRAutomaticPidTuningEvalFunc object
    // END   Code of Ctor of an eoSRAutomaticPidTuningEvalFunc object
  }

  /** Actually compute the fitness
   *
   * @param EOT & _eo the EO object to evaluate
   *                  it should stay templatized to be usable
   *                  with any fitness type
   */
  void operator()(EOT & _eo)
  {
    // test for invalid to avoid recomputing fitness of unmodified individuals
    if (_eo.invalid())
    {
      double fit = 0.0;		   // to hold fitness value
      // START Code of computation of fitness of the eoSRAutomaticPidTuning object
      std::vector<int> pid_settings = _eo.get_pid_settings();

      //setting the pid with the new values.
      _eo.set_pid_settings(pid_settings);

      //then we move and record the error.
      for(unsigned int i=0; i<1000; ++i )
      {
        _eo.execute_mvt_step(i);
        double last_error = _eo.get_last_error();

        fit += last_error*last_error;
      }
      fit /= 1000.0;

      ROS_INFO_STREAM(" Fitness = " << fit << " P:" << pid_settings[0] << " I:"<<pid_settings[1]
                      << " D:" << pid_settings[2] << " Imax:" << pid_settings[3]);

      // END   Code of computation of fitness of the eoSRAutomaticPidTuning object
      _eo.fitness(fit);
    }
  }

private:
// START Private data of an eoSRAutomaticPidTuningEvalFunc object
  //  varType anyVariable;		   // for example ...
// END   Private data of an eoSRAutomaticPidTuningEvalFunc object
};


/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/


#endif
