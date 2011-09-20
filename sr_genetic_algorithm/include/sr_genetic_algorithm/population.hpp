/**
 * @file   population.hpp
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

#ifndef _POPULATION_HPP_
#define _POPULATION_HPP_

#include <boost/ptr_container/ptr_vector.hpp>

#include "sr_genetic_algorithm/individual.hpp"
#include "sr_genetic_algorithm/termination_criterion.hpp"
#include "sr_genetic_algorithm/genetic_algorithm_parameters.hpp"
#include <boost/foreach.hpp>
#include <algorithm>
#include <vector>


#include <sr_utilities/mtrand.h>

namespace shadow_robot
{
  template <class GeneType>
  class Population
  {
  public:
    Population(std::vector<GeneType> starting_seed, unsigned int population_size, TerminationCriterion termination_criterion, GeneticAlgorithmParameters parameters)
      : ga_parameters(parameters)
    {
      drand = boost::shared_ptr<sr_utilities::MTRand>( new sr_utilities::MTRand() );

      individuals = boost::shared_ptr<std::vector<Individual<GeneType> > >( new std::vector<Individual<GeneType> >() );
      for( unsigned int i=0; i < population_size; ++i)
      {
        individuals->push_back(Individual<GeneType>(starting_seed, ga_parameters));
      }

      individuals_old = boost::shared_ptr<std::vector<Individual<GeneType> > >( new std::vector<Individual<GeneType> >() );
      for( unsigned int i=0; i < population_size; ++i)
      {
        individuals->push_back(Individual<GeneType>(starting_seed, ga_parameters));
      }

      this->termination_criterion = termination_criterion;
    };

    Population(boost::shared_ptr<std::vector<Individual<GeneType> > > individuals, TerminationCriterion termination_criterion)
    {
      drand = boost::shared_ptr<sr_utilities::MTRand>( new sr_utilities::MTRand() );

      this->individuals = individuals;
      this->termination_criterion = termination_criterion;
    };

    virtual ~Population(){};

    /**
     * Runs one cycle of the GA:
     * - selection
     * - reproduction
     * - evaluate fitness
     *
     * @return NO_CONVERGENCE if no termination reached, a TerminationReason otherwise.
     */
    TerminationCriterion::TerminationReason cycle_once()
    {    //compute the fitnesses for all the individuals.
      compute_fitnesses();
      //sort the individuals by diminishing fitnesses
      std::sort( individuals->begin(), individuals->end(), compare_fitness::sort_by_fitness<GeneType> );

      //create the new population:
      // -> set the current individuals as the parents
      // -> then select / crossover / mutate to create the new population
      // The new population has the same size as the old one.
      individuals_old.swap(individuals);
      individuals->clear();
      int index_new_indiv = 0;
      while( individuals->size() != individuals_old->size() )
      {
        std::pair<int, int> selected_indexes = select();

        crossover(selected_indexes);
        mutation(index_new_indiv);

        ++index_new_indiv;
      }

      ++ iteration_index;
      return check_termination();
    };

  protected:
    /**
     * Using a double buffer to store the individuals (swap from the recent to the old
     *  one during the reproduction cycle).
     */
    boost::shared_ptr<std::vector<Individual<GeneType> > > individuals;
    boost::shared_ptr<std::vector<Individual<GeneType> > > individuals_old;

    GeneticAlgorithmParameters ga_parameters;

    double total_fitness;

    /**
     * Select 2 individuals to be parents, using the roulette-wheel selection.
     *
     *
     * @return a pair containing the 2 indexes of the selected parents.
     */
    std::pair<int, int> select()
    {
      std::pair<int, int> selected_indexes;

      selected_indexes.first = roulette_wheel();
      selected_indexes.second = roulette_wheel();

      return selected_indexes;
    };
    int roulette_wheel()
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
    };

    ///random number generators
    boost::shared_ptr<sr_utilities::MTRand> drand;

    void compute_fitnesses()
    {
      total_fitness = 0;
      BOOST_FOREACH(Individual<GeneType> individual, individuals)
      {
        individual.compute_fitness();

        total_fitness += individual.get_fitness();

        ++ function_evaluation_index;
      }
    };

    TerminationCriterion::TerminationReason check_termination()
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
    };

    void mutation(int index)
    {
      if( drand() < ga_parameters.mutation_probability )
      {
        individuals[index].mutate();
      }
    };

    void crossover(std::pair<int, int> selected_indexes)
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
    };

    ///counts the number of iteration we did
    unsigned int iteration_index;
    ///counts the number of time we called the fitness function
    unsigned int function_evaluation_index;
    //Contains the different parameters for the termination:
    struct TerminationCriterion termination_criterion;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
