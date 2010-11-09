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
#include "measure.hpp"

#include <boost/smart_ptr.hpp>

namespace dataglove
{
class ParticleSrHand : public Particle
{
public:
    ParticleSrHand();
    ParticleSrHand( int population_size );
    ~ParticleSrHand();

    //particle filter functions
    virtual void init_model();
    virtual void prediction();
    virtual void compute_probability( boost::shared_ptr<Measure> measure );
};
}

#endif /* PARTICLE_SR_HAND_CPP_ */
