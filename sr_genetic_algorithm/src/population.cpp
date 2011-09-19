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
#include "sr_genetic_algorithm/genetic_algorithm_parameters.hpp"
#include <boost/foreach.hpp>
#include <algorithm>
#include <vector>

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
    //compute the fitnesses for all the individuals.
    compute_fitnesses();
    return check_termination();

    //sort the individuals by diminishing fitnesses
    std::sort( individuals->begin(), individuals->end(), compare_fitness::sort_by_fitness<GeneType> );

    //create the new population:
    // -> set the current individuals as the parents
    // -> then select / crossover / mutate to create the new population
    // The new population has the same size as the old one.
    individuals_old.swap(individuals);
    individuals = boost::shared_ptr<std::vector<Individual<GeneType> > >(new std::vector<Individual<GeneType> >());
    while( individuals->size() != individuals_old->size() )
    {
      std::pair<int, int> selected_indexes = select();

      crossover(selected_indexes);
      mutation(selected_indexes);
    }

    ++ iteration_index;
  }

  template <class GeneType>
  void Population<GeneType>::compute_fitnesses()
  {
    total_fitness = 0;
    BOOST_FOREACH(Individual<GeneType> individual, individuals)
    {
      individual.compute_fitness();

      total_fitness += individual.get_fitness();

      ++ function_evaluation_index;
    }
  }

  template <class GeneType>
  std::pair<int, int> Population<GeneType>::select()
  {
    std::pair<int, int> selected_indexes;

    selected_indexes.first = roulette_wheel();
    selected_indexes.second = roulette_wheel();

    return selected_indexes;
  }

  template <class GeneType>
  int Population<GeneType>::roulette_wheel()
  {
    //generate a random number
    double rand_for_roulette = drand();

    //normalize the weights and compute the accumulated fitness value
    // until you reach the random number.
    double cumulated_fitness = 0.0;
    int selected_index = 0;
    BOOST_FOREACH(Individual<GeneType> individual, individuals)
    {
      cumulated_fitness += (individual.get_fitness() / total_fitness);
      if( cumulated_fitness >= rand_for_roulette )
        return selected_index;
      selected_index += 1;
    }

    //return the last index
    return selected_index - 1;
  }

  template <class GeneType>
  void Population<GeneType>::mutation(int index)
  {
    if( drand() < ga_parameters.mutation_probability )
    {
      individuals[index].mutate();
    }
  }

  template <class GeneType>
  void Population<GeneType>::crossover(std::pair<int, int> selected_indexes)
  {
    if( drand() < ga_parameters.crossover_probability )
    {
      Individual<GeneType> new_individual( individuals_old[selected_indexes.first],
                                           individuals_old[selected_indexes.second]);

      individuals->push_back(new_individual );
    }
    else
    {
      //no crossovers -> we just keep parent A.
      individuals->push_back( individuals_old[selected_indexes.first] );
    }
  }

  template <class GeneType>
  TerminationCriterion::TerminationReason Population<GeneType>::check_termination()
  {
    if( iteration_index >= termination_criterion.max_iteration_number)
      return TerminationCriterion::MAX_ITERATION_NUMBER;
    if( function_evaluation_index >= termination_criterion.max_number_function_evaluation)
      return TerminationCriterion::MAX_NUMBER_FUNCTION_EVALUATION;

    //the individuals are ordered from the best to the worst fitness value.
    if( individuals[0].get_fitness() <= termination_criterion.best_fitness )
      return TerminationCriterion::BEST_FITNESS;

    // the population hasn't converged
    return TerminationCriterion::NO_CONVERGENCE;
  }

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
