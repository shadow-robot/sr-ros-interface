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
const unsigned int DatagloveProcessing::total_number_of_particles = 200;

DatagloveProcessing::DatagloveProcessing()
{
    for( unsigned int particle_index = 0; particle_index < total_number_of_particles; ++particle_index )
    {
        particle_cloud.push_back(new ParticleSrHand(total_number_of_particles));
    }
}

DatagloveProcessing::~DatagloveProcessing()
{

}

void DatagloveProcessing::update()
{

}
}
