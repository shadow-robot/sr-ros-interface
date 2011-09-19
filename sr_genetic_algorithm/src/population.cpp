/**
 * @file   population.cpp
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

#include "sr_genetic_algorithm/population.hpp"

namespace shadow_robot
{
  template <class GeneType>
  Population<GeneType>::Population(std::vector<GeneType> starting_seed, unsigned int population_size,
                                   TerminationCriterion termination_criterion)
  {
    individuals = boost::shared_ptr<std::vector<Individual<GeneType> > >( new std::vector<Individual<GeneType> >() );

    for( unsigned int i=0; i < population_size; ++i)
    {
      individuals->push_back(Individual<GeneType>(starting_seed));
    }

    individuals_old = boost::shared_ptr<std::vector<Individual<GeneType> > >( individuals->copy() );

    this->termination_criterion = termination_criterion;
  }

  template <class GeneType>
  Population<GeneType>::Population(boost::shared_ptr<std::vector<Individual<GeneType> > > individuals,
                                   TerminationCriterion termination_criterion)
  {
    this->individuals = individuals;
    this->termination_criterion = termination_criterion;
  }

  template <class GeneType>
  Population<GeneType>::~Population()
  {
  }

  template <class GeneType>
  TerminationCriterion::TerminationReason Population<GeneType>::cycle_once()
  {
    return TerminationCriterion::NO_CONVERGENCE;
  }

  template <class GeneType>
  void Population<GeneType>::selection()
  {
  }

  template <class GeneType>
  void Population<GeneType>::reproduction()
  {
  }

  template <class GeneType>
  TerminationCriterion::TerminationReason Population<GeneType>::check_termination()
  {
    if( iteration_index >= termination_criterion.max_iteration_number)
      return TerminationCriterion::MAX_ITERATION_NUMBER;
    if( function_evaluation_index >= termination_criterion.max_number_function_evaluation)
      return TerminationCriterion::MAX_NUMBER_FUNCTION_EVALUTION;

    //the individuals are ordered from the best to the worst fitness value.
    if( individuals[0].get_fitness() <= termination_criterion.best_fitness )
      return TerminationCriterion::BEST_FITNESS;

    return TerminationCriterion::NO_CONVERGENCE;
  }

  template <class GeneType>
  void Population<GeneType>::mutation()
  {
    return true;
  }

  template <class GeneType>
  void Population<GeneType>::crossover()
  {
  }


}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
