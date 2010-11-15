/**
 * @file particle_sr_hand.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#include "particle_sr_hand.hpp"

using namespace shadowrobot;

namespace dataglove
{
ParticleSrHand::ParticleSrHand() :
    VirtualShadowhandLibrary(), Particle()
{
    math_utils = boost::shared_ptr<math_utils::MathUtils>(new math_utils::MathUtils());

    init_model();
}

ParticleSrHand::ParticleSrHand( int population_size ) :
    VirtualShadowhandLibrary(), Particle(population_size)
{
    math_utils = boost::shared_ptr<math_utils::MathUtils>(new math_utils::MathUtils());

    init_model();
}

ParticleSrHand::ParticleSrHand( boost::ptr_vector<ParticleSrHand>::iterator particle, bool reset_weight, float average_weight ) :
    Particle(particle, reset_weight, average_weight)
{
    math_utils = boost::shared_ptr<math_utils::MathUtils>(new math_utils::MathUtils());
    joints_map_mutex.lock();
    particle->joints_map_mutex.lock();
    joints_map.insert(particle->joints_map.begin(), particle->joints_map.end());
    particle->joints_map_mutex.unlock();
    joints_map_mutex.unlock();
}

ParticleSrHand::~ParticleSrHand()
{

}

void ParticleSrHand::init_model()
{
    JointsMap::iterator it;

    joints_map_mutex.lock();
    for( it = joints_map.begin(); it != joints_map.end(); ++it )
    {
        it->second.position = math_utils->maut_random(it->second.min, it->second.max);
    }
    joints_map_mutex.unlock();
}

void ParticleSrHand::update()
{
    prediction();
    squared_weight = compute_probability();
    squared_weight *= squared_weight;
}

void ParticleSrHand::prediction()
{

}

float ParticleSrHand::compute_probability()
{
    return 0.0f;
}

std::vector<float> ParticleSrHand::get_positions()
{
    JointsMap::iterator it;
    std::vector<float> pos;

    joints_map_mutex.lock();
    for( it = joints_map.begin(); it != joints_map.end(); ++it )
    {
        pos.push_back(it->second.position);
    }
    joints_map_mutex.unlock();

    return pos;
}
}
