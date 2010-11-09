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

#include "dataglove_processing.hpp"
#include "particle_sr_hand.hpp"
#include "particle.hpp"
#include "measure.hpp"

#include <boost/smart_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#define TEST_EXPRESSION(a) EXPECT_EQ((a), meval::EvaluateMathExpression(#a))

using namespace dataglove;

TEST(DatagloveProcessing, particleFilterInitialisation)
{
    boost::shared_ptr<DatagloveProcessing> dataglove_processing(new DatagloveProcessing());

    unsigned int total_nb_particles = dataglove_processing->get_total_number_of_particles();
    float average_weight = 1.0f / ((float)total_nb_particles);

    std::vector<float> weights;
    weights = dataglove_processing->get_weights_vector();
    EXPECT_TRUE(weights.size() == total_nb_particles)<< "Expected Size = "<< total_nb_particles << " Received size = " << weights.size();
    std::vector<float>::iterator it;
    for( it = weights.begin(); it != weights.end(); ++it )
    {
        EXPECT_TRUE(*it == average_weight) << "Expected value = " << average_weight << " Received value = " << *it;
    }
}

// Run all the tests that were declared with TEST()
int main( int argc, char **argv )
{
    ros::init(argc, argv, "test_dataglove_processing");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}
