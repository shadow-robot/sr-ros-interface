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

void ParticleSrHand::update( float* sum_weights, boost::mutex* mutex_sum_weights, boost::shared_ptr<std::vector<boost::shared_ptr<Measure> > > last_measures_for_processing )
{
    prediction();
    compute_probability(last_measures_for_processing);

    mutex_sum_weights->lock();
    *sum_weights += weight;
    mutex_sum_weights->unlock();
}

void ParticleSrHand::prediction()
{

}

void ParticleSrHand::compute_probability( boost::shared_ptr<std::vector<boost::shared_ptr<Measure> > > last_measures_for_processing )
{
    std::vector<boost::shared_ptr<Measure> >::iterator measure;
    measure = last_measures_for_processing->begin();
    for( measure; measure != last_measures_for_processing->end(); ++measure )
    {
        weight *= measure->get()->compute_probability();
    }
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
