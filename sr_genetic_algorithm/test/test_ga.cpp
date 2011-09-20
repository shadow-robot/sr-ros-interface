/**
 * @file   test_ga.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep 20 10:05:50 2011
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

#include "sr_genetic_algorithm/genetic_algorithm.hpp"
#include "sr_genetic_algorithm/genetic_algorithm_parameters.hpp"
#include "sr_genetic_algorithm/termination_criterion.hpp"
#include <boost/smart_ptr.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

using namespace shadow_robot;

TEST(GeneticAlgorithm, initialization)
{
  std::vector<int> seed;
  for(unsigned int i=0; i < 10; ++i)
    seed.push_back(0);

  TerminationCriterion tc;
  tc.best_fitness = 1;
  tc.max_iteration_number = 1000;
  tc.max_number_function_evaluation = 1000000;

  boost::shared_ptr<GeneticAlgorithm<int> > ga;

  GeneticAlgorithmParameters ga_parameters;
  ga_parameters.crossover_probability = 0.5;
  ga_parameters.mutation_probability = 0.0001;
  ga_parameters.gene_max_percentage_change = 0.5;
  ga_parameters.max_mutation_percentage_rate = 0.5;


  ga = boost::shared_ptr<GeneticAlgorithm<int> >( new GeneticAlgorithm<int>(seed, 1000, tc, ga_parameters) );

  EXPECT_EQ(0,0);
}



/////////////////////
//     MAIN       //
///////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "genetic_algorithm_test");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
