/** -*- mode: c++; c-indent-level: 4; c++-member-init-indent: 8; comment-column: 35; -*-

The above line is usefulin Emacs-like editors
 */

/*
Template for simple quadratic crossover operators
=================================================

Quadratic crossover operators modify the both genotypes
*/

#ifndef eoSRAutomaticPidTuningQuadCrossover_H
#define eoSRAutomaticPidTuningQuadCrossover_H

#include <eoOp.h>

/**
 *  Always write a comment in this format before class definition
 *  if you want the class to be documented by Doxygen
 *
 * THere is NO ASSUMPTION on the class GenoypeT.
 * In particular, it does not need to derive from EO
 */
template<class GenotypeT>
class eoSRAutomaticPidTuningQuadCrossover: public eoQuadOp<GenotypeT>
{
public:
  /**
   * Ctor - no requirement
   */
// START eventually add or modify the anyVariable argument
  eoSRAutomaticPidTuningQuadCrossover()
  //  eoSRAutomaticPidTuningQuadCrossover( varType  _anyVariable) : anyVariable(_anyVariable)
  // END eventually add or modify the anyVariable argument
  {
    // START Code of Ctor of an eoSRAutomaticPidTuningEvalFunc object
    // END   Code of Ctor of an eoSRAutomaticPidTuningEvalFunc object
  }

  /// The class name. Used to display statistics
  string className() const { return "eoSRAutomaticPidTuningQuadCrossover"; }

  /**
   * eoQuad crossover - _genotype1 and _genotype2 are the (future)
   *       offspring, i.e. _copies_ of the parents, to be modified
   * @param _genotype1 The first parent
   * @param _genotype2 The second parent
   */
  bool operator()(GenotypeT& _genotype1, GenotypeT & _genotype2)
  {
    bool oneAtLeastIsModified(true);
    // START code for crossover of _genotype1 and _genotype2 objects
    std::vector<int> pid_1 = _genotype1.get_pid_settings();
    unsigned int index_to_modify1  = sr_math_utils::Random::instance().generate<unsigned int>(0, pid_1.size());
    unsigned int index_to_modify2  = sr_math_utils::Random::instance().generate<unsigned int>(0, pid_1.size());

    //make sure index_to_modify1 is smaller than index_to_modify2
    int tmp;
    if( index_to_modify1 > index_to_modify2)
    {
      tmp = index_to_modify2;
      index_to_modify2 = index_to_modify1;
      index_to_modify1 = tmp;
    }

    //if the crossover indexes wrap the whole genome, don't modify anything
    if( index_to_modify1 == 0 && index_to_modify2 == (pid_1.size() - 1) )
      oneAtLeastIsModified = false;
    else
    {
      std::vector<int> pid_2 = _genotype2.get_pid_settings();
      std::vector<int> pid1_new, pid2_new;

      //between 0 and index_1 keep the values
      for( unsigned int i=0; i<index_to_modify1; ++i)
      {
        pid1_new.push_back( pid_1[i]);
        pid2_new.push_back( pid_2[i]);
      }
      //between index_1 and index_2, invert the values
      for( unsigned int i=index_to_modify1; i<index_to_modify2; ++i)
      {
        pid1_new.push_back( pid_2[i]);
        pid2_new.push_back( pid_1[i]);
      }
      //between index_2 and end, keep the values
      for( unsigned int i=index_to_modify2; i<pid_1.size(); ++i)
      {
        pid1_new.push_back( pid_1[i]);
        pid2_new.push_back( pid_2[i]);
      }

      _genotype1.set_pid_settings(pid_1);
      _genotype2.set_pid_settings(pid_2);
    }

    /** Requirement
     * if (at least one genotype has been modified) // no way to distinguish
     *     oneAtLeastIsModified = true;
     * else
     *     oneAtLeastIsModified = false;
     */
    return oneAtLeastIsModified;
    // END code for crossover of _genotype1 and _genotype2 objects
  }

private:
// START Private data of an eoSRAutomaticPidTuningQuadCrossover object
  //  varType anyVariable;		   // for example ...
// END   Private data of an eoSRAutomaticPidTuningQuadCrossover object
};

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
