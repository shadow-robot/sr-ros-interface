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
  eoSRAutomaticPidTuningInit(std::vector<int> seed, std::vector<int> max_variations, std::string controller_topic)
  //  eoSRAutomaticPidTuningInit( varType  _anyVariable) : anyVariable(_anyVariable)
  // END eventually add or modify the anyVariable argument
  {
    // START Code of Ctor of an eoSRAutomaticPidTuningInit object
    for ( unsigned int i=0; i < seed.size(); ++i)
    {
      int min_value = seed[i] - max_variations[i];
      if( min_value < 0 )
        min_value = 0;
      int max_value = seed[i] + max_variations[i];

      min_range.push_back(min_value);
      max_range.push_back(max_value);
    }
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
    _genotype.set_pid_settings(initial_pids);
    _genotype.set_min_range(min_range);
    _genotype.set_max_range(max_range);
  }

private:
// START Private data of an eoSRAutomaticPidTuningInit object
  std::vector<int> min_range;
  std::vector<int> max_range;
// END   Private data of an eoSRAutomaticPidTuningInit object
};

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
