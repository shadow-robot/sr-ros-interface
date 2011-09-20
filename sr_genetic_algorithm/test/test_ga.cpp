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

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/function.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>

using namespace shadow_robot;

double fitness_function(std::vector<int> genome)
{
  double sum = 0.0;
  for(unsigned int i=0; i < genome.size() ; ++ i)
    sum += static_cast<double>(genome[i]);

  return 1.0 / sum;
}

void callback(std::vector<int> best_genome, double best_fitness, double average_fitness)
{
  std::cout << "best fitness: " << best_fitness << " average fitness: "<< average_fitness<< std::endl;
}

TEST(GeneticAlgorithm, initialization)
{
  std::vector<int> seed;
  for(unsigned int i=0; i < 10; ++i)
    seed.push_back(200);

  TerminationCriterion tc;
  tc.best_fitness = 1.0;
  tc.max_iteration_number = 500;
  tc.max_number_function_evaluation = 1000000;

  boost::shared_ptr<GeneticAlgorithm<int> > ga;

  GeneticAlgorithmParameters ga_parameters;
  ga_parameters.crossover_probability = 0.9;
  ga_parameters.mutation_probability = 0.0001;
  ga_parameters.gene_max_percentage_change = 0.5;
  ga_parameters.max_mutation_percentage_rate = 0.5;


  ga = boost::shared_ptr<GeneticAlgorithm<int> >( new GeneticAlgorithm<int>(seed, 1000, tc, ga_parameters,
                                                                            boost::bind(&fitness_function, _1),
                                                                            boost::bind(&callback, _1, _2, _3) ) );

  ga->run();

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
