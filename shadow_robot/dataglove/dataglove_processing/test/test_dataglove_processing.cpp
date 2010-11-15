/**
 * @file test_dataglove_processing.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief Test the Dataglove processing.
 *
 *
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "gtest/gtest_prod.h"

#include "dataglove_processing.hpp"
#include "particle_sr_hand.hpp"
#include "particle.hpp"
#include "measure.hpp"
#include <math_utils.hpp>
#include <vector>
#include <algorithm>

#include <boost/smart_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#define TEST_EXPRESSION(a) EXPECT_EQ((a), meval::EvaluateMathExpression(#a))

using namespace dataglove;

namespace dataglove
{
class DatagloveProcessingTest : public ::testing::Test
{

};
TEST(DatagloveProcessingTest, pfInitWeights)
{
    DatagloveProcessing dataglove_processing;

    unsigned int total_nb_particles = dataglove_processing.total_number_of_particles;
    float average_weight = 1.0f / ((float)total_nb_particles);

    std::vector<float> weights;
    weights = dataglove_processing.get_weights_vector();
    EXPECT_TRUE(weights.size() == total_nb_particles)<< "Expected Size = "<< total_nb_particles << " Received size = " << weights.size();
    std::vector<float>::iterator it;
    for( it = weights.begin(); it != weights.end(); ++it )
    {
        EXPECT_TRUE(*it == average_weight) << "Expected value = " << average_weight << " Received value = " << *it;
    }
}

TEST(DatagloveProcessingTest, pfInitPos)
{
    DatagloveProcessing dataglove_processing;
    /*
     * Test positions are random
     */

    std::vector<std::vector<float> > all_part_positions;
    all_part_positions = dataglove_processing.get_particle_positions_vector();

    std::vector<float> first_pos = all_part_positions[0];

    EXPECT_TRUE(first_pos.size() == 28)<< "Wrong number of joints: " << first_pos.size();

    // ignore the first as it's the reference
    for(unsigned int index=1; index < all_part_positions.size(); ++index)
    {
        //compare the 2 vectors
        bool result = std::equal(first_pos.begin(), first_pos.end(), all_part_positions[index].begin());
        // they should be different
        EXPECT_FALSE(result) << "Two position vectors are exactly equal.";
    }
}

TEST(DatagloveProcessingTest, sortByWeights)
{
    math_utils::MathUtils math_utils;
    boost::ptr_vector<ParticleSrHand> test_cloud;

    float rand = 0.0f;
    for( unsigned int i = 0; i < 1000; ++i )
    {
        ParticleSrHand* part = new ParticleSrHand(1000);
        rand = math_utils.maut_random(0.0f, 10.0f);
        part->set_weight(rand);

        test_cloud.push_back(part);
    }

    CompareParticleWeights compare_particle_weights;
    test_cloud.sort(compare_particle_weights);

    boost::ptr_vector<ParticleSrHand>::iterator particle;
    particle = test_cloud.begin();
    float current_weight = particle->get_weight();
    particle++;
    for( particle; particle != test_cloud.end(); ++particle )
    {
        EXPECT_TRUE(particle->get_weight() >= current_weight)<< "Badly sorted: "
        <<particle->get_weight() <<" < " <<current_weight;
    }

}

TEST(DatagloveProcessingTest, resampling)
{
    DatagloveProcessing dataglove_processing;
    math_utils::MathUtils math_utils;

    float rand = 0.0f;
    for( unsigned int i = 0; i < dataglove_processing.total_number_of_particles; ++i )
    {
        ParticleSrHand* part = new ParticleSrHand(dataglove_processing.total_number_of_particles);
        rand = math_utils.maut_random(0.0f, 0.001f);
        part->set_weight(rand);

        dataglove_processing.particle_cloud->push_back(part);
    }

    int different_particles_before_resampling = 0;
    boost::ptr_vector<ParticleSrHand>::iterator particle = dataglove_processing.particle_cloud->begin();
    boost::ptr_vector<ParticleSrHand>::iterator old_particle = dataglove_processing.particle_cloud->begin();

    particle ++;
    for( particle; particle != dataglove_processing.particle_cloud->end(); ++particle )
    {
        if(particle == old_particle)
            different_particles_before_resampling ++;
    }

    int resampled = dataglove_processing.resampling();
    EXPECT_TRUE(resampled == 1)<< "No resampling";
    EXPECT_TRUE(dataglove_processing.particle_cloud->size() == dataglove_processing.total_number_of_particles)
    << "Wrong cloud size after resampling: " << dataglove_processing.particle_cloud->size() << "(expected: "
    <<dataglove_processing.total_number_of_particles <<" )";

    /*
     *check if the cloud has really been resampled:
     */
    //check if there are enough different particles
    int different_particles_after_resampling = 0;
    //check if there are less different particles than before.
}


TEST(DatagloveProcessingTest, updateCycle)
{
    DatagloveProcessing dataglove_processing;
    int result = dataglove_processing.update_cycle();
    EXPECT_TRUE(result == 0 || result == 1) << "Problem encountered while doing the update.";
}

bool test_random( float min, float max, int iteration )
{
    math_utils::MathUtils math_utils;
    float result = 0.0f;
    for( unsigned int i = 0; i < iteration; ++i )
    {
        result = math_utils.maut_random(min, max);
        if( result < min )
        return false;
        if( result > max )
        return false;
    }
    return true;
}

TEST(MathUtils, random)
{
    bool is_in_range;
    is_in_range = test_random(-10.0f, 10.0f, 1000);
    EXPECT_TRUE(is_in_range)<< "Range -10, 10";
    is_in_range = test_random(0.0f, 10.0f, 1000);
    EXPECT_TRUE(is_in_range) << "Range 0, 10";
    is_in_range = test_random(-10.0f, 0.0f, 1000);
    EXPECT_TRUE(is_in_range) << "Range -10, 0";
    is_in_range = test_random(-20.0f, -2.0f, 1000);
    EXPECT_TRUE(is_in_range) << "Range -20, 2";
    is_in_range = test_random(2.0f, 20.0f, 1000);
    EXPECT_TRUE(is_in_range) << "Range 2, 20";
    is_in_range = test_random(0.4f, 0.4f, 1000);
    EXPECT_TRUE(is_in_range) << "Range 0.4, 0.4";
}
}
// Run all the tests that were declared with TEST()
int main( int argc, char **argv )
{
    ros::init(argc, argv, "test_dataglove_processing");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}
