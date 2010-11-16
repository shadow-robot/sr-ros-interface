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
    ParticleSrHand( boost::ptr_vector<ParticleSrHand>::iterator particle, bool reset_weight, float average_weight );
    ~ParticleSrHand();

    //particle filter functions
    virtual void init_model();

    /**
     * Runs the update cycle: first prediction()
     * then compute_probability(last_measure)
     *
     * @param sum_weights a pointer to the Sum of weights for the whole cloud
     * @param mutex_squared_weights a pointer to the mutex used to block this variable
     */
    virtual void update(float* sum_weights, boost::mutex* mutex_sum_weights);

    std::vector<float> get_positions();
protected:
    boost::shared_ptr<math_utils::MathUtils> math_utils;

    virtual void prediction();
    virtual void compute_probability();

};
}

#endif /* PARTICLE_SR_HAND_CPP_ */
