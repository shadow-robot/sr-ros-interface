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

Particle::~Particle()
{

}

void Particle::init_weight( int population_size )
{
    weight = 1.0f / ((float)population_size);
}

float Particle::get_weight()
{
    return weight;
}

}//end namespace
