/**
 * @file particle.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#include "particle.hpp"
#include <ros/ros.h>

namespace dataglove
{
Particle::Particle()
{
    init_weight(1);
}

Particle::Particle( int population_size )
{
    init_weight(population_size);
}

Particle::Particle( boost::ptr_vector<Particle>::iterator particle, bool reset_weight, float average_weight )
{
    if( reset_weight )
        weight = average_weight;
    else
        weight = particle->weight;
}

Particle::~Particle()
{

}

void Particle::init_weight( int population_size )
{
    weight = 1.0f / ((float)population_size);
}

float Particle::get_weight() const
{
    return weight;
}

void Particle::set_weight( float new_weight )
{
    weight = new_weight;
}

void Particle::set_last_measure( boost::shared_ptr<Measure> last_measure )
{
    last_measure = last_measure;
}

}//end namespace
