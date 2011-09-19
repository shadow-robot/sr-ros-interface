/**
 * @file   individual.cpp
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

#include "sr_genetic_algorithm/individual.hpp"
#include <boost/foreach.hpp>

namespace shadow_robot
{
  template <class GeneType>
  const GeneType Individual<GeneType>::gene_max_percentage_change = 0.5;


  template <class GeneType>
  Individual<GeneType>::Individual(std::vector<GeneType> starting_seed, double max_mutation_percentage_rate)
    : max_mutation_percentage_rate(max_mutation_percentage_rate)
  {
    BOOST_FOREACH(GeneType gene, starting_seed)
    {
      /**
       * Generates a number that's a random number between the
       *  given value - gene_max_percentage_change and the given
       *  value + gene_max_percentage_change.
       */
      gene = static_cast<GeneType>( gene - gene_max_percentage_change + (drand() * static_cast<GeneType>(2) * gene_max_percentage_change) );

      /**
       * Clamp it in the correct region: we want all the numbers
       * to be positive.
       */
      if( gene < static_cast<GeneType>(0) )
        gene = -gene;

      genome.push_back( new GeneType( gene ) );
    }

    range_rand = boost::shared_ptr<sr_utilities::MTRangedRand<unsigned int> >(new sr_utilities::MTRangedRand<unsigned int>(0, genome.size()));
  }

  template <class GeneType>
  Individual<GeneType>::~Individual()
  {
  }

  template <class GeneType>
  void Individual<GeneType>::mutate()
  {
    int index_to_mutate = range_rand();

    GeneType max_mutation = max_mutation_percentage_rate * genome[index_to_mutate];

    //mutate the gene
    genome[index_to_mutate] = static_cast<GeneType>( genome[index_to_mutate] - max_mutation + 2.0 * drand() * max_mutation );
    //clamp to positive values
    if( genome[index_to_mutate] < 0 )
      genome[index_to_mutate] = -genome[index_to_mutate];
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
