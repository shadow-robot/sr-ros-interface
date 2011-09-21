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

#include <sr_utilities/sr_math_utils.hpp>

#include <iostream>

namespace shadow_robot
{
  template <class GeneType>
  class Individual
  {
  public:
    Individual(std::vector<GeneType> starting_seed, GeneticAlgorithmParameters parameters,
               boost::function<double( std::vector<GeneType> )> fitness_function,
               boost::shared_ptr<sr_math_utils::RandomDouble> drand,
               boost::shared_ptr<sr_math_utils::RandomRangedInt> ranged_rand)
      : fitness_function(fitness_function), ga_parameters(parameters),
        drand(drand), ranged_rand(ranged_rand)
    {
      BOOST_ASSERT(ranged_rand != 0);

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

      /*
      std::cout << std::endl;
      std::cout << " new genome: ";
      for(unsigned int i=0; i < genome.size(); ++i)
        std::cout << genome[i] << " ";
      std::cout << std::endl;
      */
    };

    /**
     * Constructors for the crossover:
     *  Creates a new individual from the 2 given individuals.
     */
    Individual(const Individual<GeneType>& a, const Individual<GeneType>& b)
      : drand(a.drand), ranged_rand(a.ranged_rand)
    {
      BOOST_ASSERT(ranged_rand != 0);

      fitness_function = a.fitness_function;
      fitness = a.fitness;
      //create the new individual, using a crossover.
      genome = a.genome;

      int index_for_crossover = ranged_rand->generate();

      std::cout << " ----- " << std::endl;
      std::cout << " genomes before crossover (at "<< index_for_crossover<<"): ";
      for(unsigned int i=0; i < genome.size(); ++i)
        std::cout << genome[i] << " ";
      std::cout << " / ";
      for(unsigned int i=0; i < genome.size(); ++i)
        std::cout << genome[i] << " ";
      std::cout << std::endl;

      for(unsigned int i=index_for_crossover; i < genome.size(); ++i)
        genome[i] = b.genome[i];

      std::cout << " genome after crossover: ";
      for(unsigned int i=0; i < genome.size(); ++i)
        std::cout << genome[i] << " ";
      std::cout << std::endl;

    };

    /**
     * Copy Constructor for the crossover:
     *  Creates a new individual but keep the same genome.
     */
    Individual(const Individual<GeneType>& a)
      : drand(a.drand), ranged_rand(a.ranged_rand)
    {
      BOOST_ASSERT(ranged_rand != 0);

      fitness_function = a.fitness_function;
      //copy the fitness
      fitness = a.fitness;
      //create the new individual, using a crossover.
      genome = a.genome;

      /*
      std::cout << " ----- " << std::endl;
      std::cout << " copied genome: ";
      for(unsigned int i=0; i < genome.size(); ++i)
        std::cout << genome[i] << " ";
      std::cout << std::endl;
      */
    };


    virtual ~Individual(){};

    /**
     * Randomly mutates a gene of this individual.
     */
    void mutate()
    {
      BOOST_ASSERT(ranged_rand != 0);
      int index_to_mutate = ranged_rand->generate();

      std::cout << "mutation on "<< index_to_mutate << " / " << genome.size() << " gene was: " << genome[index_to_mutate] << " -> ";

      GeneType max_mutation = max_mutation_percentage_rate * genome[index_to_mutate];

      //mutate the gene
      genome[index_to_mutate] = static_cast<GeneType>( genome[index_to_mutate] - max_mutation + 2.0 * drand->generate() * max_mutation );

      std::cout << genome[index_to_mutate] << std::endl;

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

    std::vector<GeneType> genome;

  protected:
    double fitness;

    boost::function<double( std::vector<GeneType> )> fitness_function;

    GeneticAlgorithmParameters ga_parameters;

    double max_mutation_percentage_rate;

    ///random number generators
    boost::shared_ptr<sr_math_utils::RandomDouble> drand;
    boost::shared_ptr<sr_math_utils::RandomRangedInt> ranged_rand;
  };

  /**
   * Used to sort the vector of individuals
   * with their fitnesses.
   *
   * @return true if fitness Individual A < fitness Individual B.
   */
  template <class GeneType>
  struct greater
  {
    bool operator() (const Individual<GeneType>& x, const Individual<GeneType>& y) const
    {
      return x.get_fitness() > y.get_fitness();
    }
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif
