/**
 * @file dataglove_processing.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#include "dataglove_processing.hpp"

namespace dataglove
{
const unsigned int DatagloveProcessing::total_number_of_particles = 1;

DatagloveProcessing::DatagloveProcessing() :
    nh_tilde("~"), update_rate(0.0)
{
    particle_cloud.reserve(total_number_of_particles);

    for( unsigned int particle_index = 0; particle_index < total_number_of_particles; ++particle_index )
    {
        particle_cloud.push_back(new ParticleSrHand(total_number_of_particles));
    }

    // set update frequency
    double update_freq;
    nh_tilde.param("update_frequency", update_freq, 20.0);
    update_rate = ros::Rate(update_freq);
}

DatagloveProcessing::~DatagloveProcessing()
{

}

void DatagloveProcessing::update()
{
    boost::ptr_vector<ParticleSrHand>::iterator particle;
    for( particle = particle_cloud.begin(); particle != particle_cloud.end(); ++particle )
    {
        particle->prediction();
        particle->compute_probability(last_measure);
    }
    resampling();

    ros::spinOnce();
    update_rate.sleep();
}

void DatagloveProcessing::resampling()
{

}

std::vector<float> DatagloveProcessing::get_weights_vector()
{
    std::vector<float> weights;
    boost::ptr_vector<ParticleSrHand>::iterator particle;
    for( particle = particle_cloud.begin(); particle != particle_cloud.end(); ++particle )
    {
        weights.push_back(particle->get_weight());
    }
    return weights;
}

std::vector<std::vector<float> > DatagloveProcessing::get_particle_positions_vector()
{
    std::vector<std::vector<float> > all_part_positions;
    boost::ptr_vector<ParticleSrHand>::iterator particle;
    for( particle = particle_cloud.begin(); particle != particle_cloud.end(); ++particle )
    {
        std::vector<float> part_pos;
        part_pos = particle->get_positions();
        all_part_positions.push_back(part_pos);
    }
    return all_part_positions;
}

unsigned int DatagloveProcessing::get_total_number_of_particles()
{
    return total_number_of_particles;
}
}//end namespace
