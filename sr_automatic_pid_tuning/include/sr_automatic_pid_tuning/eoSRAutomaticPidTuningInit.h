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
  eoSRAutomaticPidTuningInit()
  //  eoSRAutomaticPidTuningInit( varType  _anyVariable) : anyVariable(_anyVariable)
// END eventually add or modify the anyVariable argument
  {
    // START Code of Ctor of an eoSRAutomaticPidTuningInit object
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
  }

private:
// START Private data of an eoSRAutomaticPidTuningInit object
  //  varType anyVariable;		   // for example ...
// END   Private data of an eoSRAutomaticPidTuningInit object
};

#endif
