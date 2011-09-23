/** -*- mode: c++; c-indent-level: 4; c++-member-init-indent: 8; comment-column: 35; -*-

The above line is usefulin Emacs-like editors
 */

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
	double fit;		   // to hold fitness value
    // START Code of computation of fitness of the eoSRAutomaticPidTuning object
	// fit = blablabla
    // END   Code of computation of fitness of the eoSRAutomaticPidTuning object
	_eo.fitness(fit);
      }
  }

private:
// START Private data of an eoSRAutomaticPidTuningEvalFunc object
  //  varType anyVariable;		   // for example ...
// END   Private data of an eoSRAutomaticPidTuningEvalFunc object
};


#endif
