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

#ifndef _GENETIC_ALGORITHM_HPP_
#define _GENETIC_ALGORITHM_HPP_

#include "sr_genetic_algorithm/population.hpp"
#include "sr_genetic_algorithm/termination_criterion.hpp"

#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>

#include <iostream>

namespace shadow_robot
{
  template <class GeneType>
  class GeneticAlgorithm
  {
  public:
    GeneticAlgorithm(std::vector<GeneType> starting_seed, unsigned int population_size,
                     TerminationCriterion termination_criterion, GeneticAlgorithmParameters parameters,
                     boost::function<double()> fitness_function)
      : ga_parameters(parameters)
    {
      population = boost::shared_ptr<Population<GeneType> >(new Population<GeneType>(starting_seed, population_size,
                                                                                     termination_criterion, ga_parameters,
                                                                                     fitness_function));
    };

    virtual ~GeneticAlgorithm()
    {};

    TerminationCriterion::TerminationReason run()
    {
      thread_ga = boost::shared_ptr<boost::thread>( new boost::thread( boost::bind( &GeneticAlgorithm<GeneType>::iterate_cycles, this ) ) );
      thread_ga->join();
      return TerminationCriterion::NO_CONVERGENCE;
    };

    bool pause()
    {
      return true;
    };

    bool stop()
    {
      return true;
    };

    void iterate_cycles()
    {
      for(unsigned int i=0; i<5; ++i)
      {
        population->cycle_once();
        sleep(1);
      }
    };

  protected:
    boost::shared_ptr<Population<GeneType> > population;
    GeneticAlgorithmParameters ga_parameters;

    boost::shared_ptr<boost::thread> thread_ga;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
