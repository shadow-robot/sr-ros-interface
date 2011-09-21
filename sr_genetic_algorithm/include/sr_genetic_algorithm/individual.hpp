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
               boost::function<double( std::vector<GeneType> )> fitness_function)
      : fitness_function(fitness_function), ga_parameters(parameters)
    {
      BOOST_FOREACH(GeneType gene, starting_seed)
      {
        /**
         * Generates a number that's a random number between the
         *  given value - gene_max_percentage_change and the given
         *  value + gene_max_percentage_change.
         */
        GeneType min = gene - ga_parameters.gene_max_percentage_change * gene;
        GeneType max = gene + ga_parameters.gene_max_percentage_change * gene;
        gene = sr_math_utils::Random::instance().generate<GeneType>(min, max);

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
      : ga_parameters(a.ga_parameters)
    {
      fitness_function = a.fitness_function;
      fitness = a.fitness;
      //create the new individual, using a crossover.
      genome = a.genome;

      int index_for_crossover = sr_math_utils::Random::instance().generate<int>(0, genome.size());

      std::cout << " ----- " << std::endl;
      std::cout << " genomes before crossover (at "<< index_for_crossover<<"): ";
      for(unsigned int i=0; i < genome.size(); ++i)
        std::cout << genome[i] << " ";
      std::cout << " / ";
      for(unsigned int i=0; i < genome.size(); ++i)
        std::cout << b.genome[i] << " ";
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
      : ga_parameters(a.ga_parameters)
    {
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
      int index_to_mutate = sr_math_utils::Random::instance().generate<int>(0, genome.size());

      std::cout << "mutation on "<< index_to_mutate << " / " << genome.size() << std::endl;
      std::cout << " gene was: " << genome[index_to_mutate] << " -> ";

      GeneType max_mutation = ga_parameters.max_mutation_percentage_rate * genome[index_to_mutate];
      GeneType min_new_gene = genome[index_to_mutate] - max_mutation;
      GeneType max_new_gene = genome[index_to_mutate] + max_mutation;
      //mutate the gene
      genome[index_to_mutate] = sr_math_utils::Random::instance().generate<GeneType>(min_new_gene, max_new_gene);

      std::cout << genome[index_to_mutate] << "( between " << min_new_gene <<" and "<< max_new_gene << " / max: " << max_mutation<< " ; " << ga_parameters.max_mutation_percentage_rate<<")" <<std::endl;

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
