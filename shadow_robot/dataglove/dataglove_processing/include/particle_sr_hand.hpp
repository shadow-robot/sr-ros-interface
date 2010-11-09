/**
 * @file particle_sr_hand.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#ifndef PARTICLE_SR_HAND_CPP_
#define PARTICLE_SR_HAND_CPP_

#include "particle.hpp"

namespace dataglove
{
class ParticleSrHand : Particle
{
public:
    ParticleSrHand();
    ParticleSrHand( int population_size );
    ~ParticleSrHand();

    //particle filter functions
    virtual void init_model();
    virtual void prediction();
    virtual void update( Measure measure );
};
}

#endif /* PARTICLE_SR_HAND_CPP_ */
