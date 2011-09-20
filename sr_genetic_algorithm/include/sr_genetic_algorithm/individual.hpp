/**
 * @file   individual.hpp
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

#ifndef _INDIVIDUAL_HPP_
#define _INDIVIDUAL_HPP_

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>

#include "sr_genetic_algorithm/genetic_algorithm_parameters.hpp"
#include <sr_utilities/mtrand.h>

#include <iostream>

namespace shadow_robot
{
  template <class GeneType>
  class Individual
  {
  public:
    Individual(std::vector<GeneType> starting_seed, GeneticAlgorithmParameters parameters,
               boost::function<double( std::vector<GeneType> )> fitness_function)
      : fitness_function(fitness_function), ga_parameters(parameters)
    {
      drand = boost::shared_ptr<sr_utilities::MTRand>( new sr_utilities::MTRand() );
      range_rand = boost::shared_ptr<sr_utilities::MTRangedRand<unsigned int> >(new sr_utilities::MTRangedRand<unsigned int>(0, genome.size()));

      BOOST_FOREACH(GeneType gene, starting_seed)
      {
        /**
         * Generates a number that's a random number between the
         *  given value - gene_max_percentage_change and the given
         *  value + gene_max_percentage_change.
         */
        gene = static_cast<GeneType>( gene - ga_parameters.gene_max_percentage_change * gene + (drand->generate() * static_cast<GeneType>(2) * ga_parameters.gene_max_percentage_change * gene) );

        /**
         * Clamp it in the correct region: we want all the numbers
         * to be positive.
         */
        if( gene < static_cast<GeneType>(0) )
          gene = -gene;

        genome.push_back( gene );
      }
    };

    /**
     * Constructors for the crossover:
     *  Creates a new individual from the 2 given individuals.
     */
    Individual(const Individual<GeneType>& a, const Individual<GeneType>& b)
    {
      drand = boost::shared_ptr<sr_utilities::MTRand>( new sr_utilities::MTRand() );
      range_rand = boost::shared_ptr<sr_utilities::MTRangedRand<unsigned int> >(new sr_utilities::MTRangedRand<unsigned int>(0, genome.size()));
      fitness_function = a.fitness_function;

      //create the new individual, using a crossover.
      genome = a.genome;

      int index_for_crossover = range_rand->generate();
      for(unsigned int i=index_for_crossover; i < genome.size(); ++i)
        genome[i] = b.genome[i];
    };

    /**
     * Copy Constructor for the crossover:
     *  Creates a new individual but keep the same genome.
     */
    Individual(const Individual<GeneType>& a)
    {
      drand = boost::shared_ptr<sr_utilities::MTRand>( new sr_utilities::MTRand() );
      range_rand = boost::shared_ptr<sr_utilities::MTRangedRand<unsigned int> >(new sr_utilities::MTRangedRand<unsigned int>(0, genome.size()));
      fitness_function = a.fitness_function;

      //create the new individual, using a crossover.
      genome = a.genome;
    };


    virtual ~Individual(){};

    /**
     * Randomly mutates a gene of this individual.
     */
    void mutate()
    {
      int index_to_mutate = range_rand->generate();

      GeneType max_mutation = max_mutation_percentage_rate * genome[index_to_mutate];

      //mutate the gene
      genome[index_to_mutate] = static_cast<GeneType>( genome[index_to_mutate] - max_mutation + 2.0 * drand->generate() * max_mutation );
      //clamp to positive values
      if( genome[index_to_mutate] < 0 )
        genome[index_to_mutate] = -genome[index_to_mutate];
    };

    void compute_fitness()
    {
      fitness = fitness_function( genome );
    };

    /**
     * Returns the fitness for this individual.
     * @return fitness value
     */
    double get_fitness() const
    {
      return fitness;
    };

    std::vector<GeneType> get_genome() const
    {
      return genome;
    };

    /**
     * Used to sort the vector of individuals
     * with their fitnesses.
     *
     * @return true if fitness Individual A < fitness Individual B.
     */
    bool operator< ( const Individual<GeneType>& b ) const
    {
      //std::cout << "a: " << &this << " b: "<< &b << std::endl;
      return get_fitness() < b.get_fitness();
    };
  protected:
    double fitness;
    boost::function<double( std::vector<GeneType> )> fitness_function;

    GeneticAlgorithmParameters ga_parameters;

    double max_mutation_percentage_rate;

    std::vector<GeneType> genome;

    ///random number generators
    boost::shared_ptr<sr_utilities::MTRand> drand;
    boost::shared_ptr<sr_utilities::MTRangedRand<unsigned int> > range_rand;
  };

}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
