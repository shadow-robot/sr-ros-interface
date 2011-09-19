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

#include <sr_utilities/mtrand.h>

namespace shadow_robot
{
  template <class GeneType>
  class Population
  {
  public:
    Population(std::vector<GeneType> starting_seed, unsigned int population_size, TerminationCriterion termination_criterion);
    Population(boost::shared_ptr<std::vector<Individual<GeneType> > > individuals, TerminationCriterion termination_criterion);
    virtual ~Population();

    /**
     * Runs one cycle of the GA:
     * - selection
     * - reproduction
     * - evaluate fitness
     *
     * @return NO_CONVERGENCE if no termination reached, a TerminationReason otherwise.
     */
    TerminationCriterion::TerminationReason cycle_once();

  protected:
    /**
     * Using a double buffer to store the individuals (swap from the recent to the old
     *  one during the reproduction cycle).
     */
    boost::shared_ptr<std::vector<Individual<GeneType> > > individuals, individuals_old;

    GeneticAlgorithmParameters ga_parameters;

    double total_fitness;

    /**
     * Select 2 individuals to be parents, using the roulette-wheel selection.
     *
     *
     * @return a pair containing the 2 indexes of the selected parents.
     */
    std::pair<int, int> select();
    int roulette_wheel();

    ///random number generators
    sr_utilities::MTRand drand;

    void compute_fitnesses();
    TerminationCriterion::TerminationReason check_termination();

    void mutation(int index);
    void crossover(std::pair<int, int> selected_indexes);

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
