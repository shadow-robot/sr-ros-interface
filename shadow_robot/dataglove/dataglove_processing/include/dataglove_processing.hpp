/**
 * @file dataglove_processing.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#ifndef DATAGLOVE_PROCESSING_HPP_
#define DATAGLOVE_PROCESSING_HPP_

#include <ros/ros.h>
#include <vector>

#include <gtest/gtest_prod.h>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/smart_ptr.hpp>

#include <math_utils.hpp>
#include "particle_sr_hand.hpp"
#include "measure.hpp"

namespace dataglove
{
struct CompareParticleWeights : std::binary_function<ParticleSrHand, ParticleSrHand, bool>
{
    CompareParticleWeights()
    {
    }
    bool operator()( const ParticleSrHand& v1, const ParticleSrHand& v2 ) const
    {
        return (v1.get_weight() < v2.get_weight());
    }
};

class DatagloveProcessing
{
public:
    DatagloveProcessing();
    ~DatagloveProcessing();

    void update();

    //accessors
    std::vector<float> get_weights_vector();
    std::vector<std::vector<float> > get_particle_positions_vector();

    //consts
    static const unsigned int total_number_of_particles;
    static const float average_weight;
    static const unsigned int n_min;

private:
    //ROS stuff
    ros::NodeHandle nh, nh_tilde;
    /// the rate at which the update loop of the particle filter is called
    ros::Rate update_rate;

    //type
    typedef boost::ptr_vector<ParticleSrHand>::iterator part_it_t;

    //variables
    boost::ptr_vector<ParticleSrHand> particle_cloud;
    boost::shared_ptr<math_utils::MathUtils> math_utils;
    boost::shared_ptr<Measure> last_measure;
    //number of efficient particles (weight is big enough)
    unsigned int n_eff;
    //comparison structure
    CompareParticleWeights compare_particle_weights;

    //functions
    /**
     * Resample the point cloud using stochastic Universal Sampling(SUS).
     */
    void resampling();

};
}

#endif /* DATAGLOVE_PROCESSING_HPP_ */
