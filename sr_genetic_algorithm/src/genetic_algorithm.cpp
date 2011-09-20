/**
 * @file   genetic_algorithm.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Fri Sep 16 15:09:56 2011
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
*
 * @brief
 *
 *
 */

#include "sr_genetic_algorithm/genetic_algorithm.hpp"

namespace shadow_robot
{
  template <class GeneType>
  GeneticAlgorithm<GeneType>::GeneticAlgorithm(std::vector<GeneType> starting_seed, unsigned int population_size, TerminationCriterion termination_criterion)
  {
    population = boost::shared_ptr<Population<GeneType> >(new Population<GeneType>(starting_seed, population_size, termination_criterion));
  }

  template <class GeneType>
  GeneticAlgorithm<GeneType>::~GeneticAlgorithm()
  {
  }

  template <class GeneType>
  void GeneticAlgorithm<GeneType>::iterate_cycles()
  {
    while( 1 )
    {
      sleep(1);
    }
  }

  template <class GeneType>
  TerminationCriterion::TerminationReason GeneticAlgorithm<GeneType>::run()
  {
    thread_ga = boost::shared_ptr<boost::thread>( new boost::thread( boost::bind( &GeneticAlgorithm<GeneType>::iterate_cycles, this ) ) );
    return TerminationCriterion::NO_CONVERGENCE;
  }

  template <class GeneType>
  bool GeneticAlgorithm<GeneType>::pause()
  {
    return true;
  }

  template <class GeneType>
  bool GeneticAlgorithm<GeneType>::stop()
  {
    return true;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
