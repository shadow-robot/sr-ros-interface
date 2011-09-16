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

namespace shadow_robot
{
  class Population
  {
  public:
    Population(unsigned int genome_size; unsigned int population_size);
    Population(boost::ptr_vector<Individual> individuals);
    virtual ~Population();

    bool run();

  protected:
    boost::ptr_vector<Individual> individuals;

    void selection();
    void reproduction();
    bool termination();

    void mutation();
    void crossover();

  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
