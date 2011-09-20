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
#include <boost/function.hpp>
#include <algorithm>
#include <vector>

#include <iostream>

#include <sr_utilities/mtrand.h>

namespace shadow_robot
{

  template <class GeneType>
  class Population
  {
  public:
    Population(std::vector<GeneType> starting_seed, unsigned int population_size,
               TerminationCriterion termination_criterion, GeneticAlgorithmParameters parameters,
               boost::function<double( std::vector<GeneType> )> fitness_function,
               boost::function<void(std::vector<int>, double, double)> callback_function)
      : ga_parameters(parameters), callback_function(callback_function)
    {
      drand = boost::shared_ptr<sr_utilities::MTRand>( new sr_utilities::MTRand() );

      individuals = boost::shared_ptr<std::vector<Individual<GeneType> > >( new std::vector<Individual<GeneType> >() );
      for( unsigned int i=0; i < population_size; ++i)
      {
        individuals->push_back(Individual<GeneType>(starting_seed, ga_parameters, fitness_function));
      }

      individuals_old = boost::shared_ptr<std::vector<Individual<GeneType> > >( new std::vector<Individual<GeneType> >() );
      for( unsigned int i=0; i < population_size; ++i)
      {
        individuals_old->push_back(Individual<GeneType>(starting_seed, ga_parameters, fitness_function));
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

      std::cout << "before sorting: " << std::endl;
      for(unsigned int i=0; i < individuals->size(); ++i)
        std::cout << individuals->at(i).get_fitness() << " ";
      std::cout << std::endl;

      //TODO: Fix the sort function
      //sort the individuals by diminishing fitnesses
      std::sort( individuals->begin(), individuals->end(), greater<GeneType>() );

      std::cout << " ... finished sorting: ";
      for(unsigned int i=0; i < individuals->size(); ++i)
        std::cout << individuals->at(i).get_fitness() << " ";
      std::cout << std::endl;

      callback_function( individuals->begin()->get_genome(),
                         individuals->begin()->get_fitness(),
                         average_fitness);

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

        //std::cout << " selected for crossover: " << selected_indexes.first << "," << selected_indexes.second << " selected for mutation: "<<index_new_indiv <<" ("<< individuals->size() << ","<< individuals_old->size() <<")" <<std::endl;

        if( selected_indexes.first == -1 || selected_indexes.second == -1 )
        {
          std::cout << "WARNING: got an index of -1 while computing the indexes for crossover" << std::endl;
        }
        else
        {
          mutation(index_new_indiv);
          ++index_new_indiv;
        }
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
      double rand_for_roulette = drand->generate();

      //normalize the weights and compute the accumulated fitness value
      // until you reach the random number.
      double cumulated_fitness = 0.0;
      int selected_index = 0;
      for(unsigned int i=0; i < individuals_old->size(); ++i)
      {
        cumulated_fitness += (individuals_old->at(i).get_fitness() / total_fitness);

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
      total_fitness = 0.0;
      for(unsigned int i=0; i< individuals->size(); ++i)
      {
        individuals->at(i).compute_fitness();

        total_fitness += individuals->at(i).get_fitness();

        ++ function_evaluation_index;
      }

      average_fitness = total_fitness / static_cast<double>( individuals->size() );
    };

    TerminationCriterion::TerminationReason check_termination()
    {
      if( iteration_index >= termination_criterion.max_iteration_number)
        return TerminationCriterion::MAX_ITERATION_NUMBER;
      if( function_evaluation_index >= termination_criterion.max_number_function_evaluation)
        return TerminationCriterion::MAX_NUMBER_FUNCTION_EVALUATION;

      //the individuals are ordered from the best to the worst fitness value.
      if( individuals->begin()->get_fitness() <= termination_criterion.best_fitness )
        return TerminationCriterion::BEST_FITNESS;

      // the population hasn't converged
      return TerminationCriterion::NO_CONVERGENCE;
    };

    void mutation(int index)
    {
      if( drand->generate() < ga_parameters.mutation_probability )
      {
        individuals->at(index).mutate();
      }
    };

    void crossover(std::pair<int, int> selected_indexes)
    {
      if( drand->generate() < ga_parameters.crossover_probability )
      {
        Individual<GeneType> new_individual( individuals_old->at(selected_indexes.first),
                                             individuals_old->at(selected_indexes.second) );

        individuals->push_back(new_individual );
      }
      else
      {
        //no crossovers -> we just keep parent A.
        //Individual<GeneType> new_individual( individuals_old->at(selected_indexes.first) );
        //individuals->push_back( new_individual );
      }
    };

    boost::function<void(std::vector<int>, double, double)> callback_function;

    double average_fitness;

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
