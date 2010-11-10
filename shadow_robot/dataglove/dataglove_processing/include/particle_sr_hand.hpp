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

#include <gtest/gtest_prod.h>

#include <sr_hand/hand/virtual_shadowhand_library.h>
#include <math_utils.hpp>

#include <boost/smart_ptr.hpp>
#include <vector>

using namespace shadowrobot;

namespace dataglove
{
class ParticleSrHand : public virtual VirtualShadowhandLibrary, public virtual Particle
{
public:
    ParticleSrHand();
    ParticleSrHand( int population_size );
    ~ParticleSrHand();

    //particle filter functions
    virtual void init_model();
    virtual void prediction();
    virtual void compute_probability( boost::shared_ptr<Measure> measure );

    std::vector<float> get_positions();
private:
    boost::shared_ptr<math_utils::MathUtils> math_utils;

};
}

#endif /* PARTICLE_SR_HAND_CPP_ */
