/**
 * @file particle.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief Generic particle from which our particles will inherit.
 *
 *
 */

#ifndef PARTICLE_HPP_
#define PARTICLE_HPP_

#include "measure.hpp"
#include <boost/smart_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace dataglove
{
class Particle : boost::noncopyable
{
public:
    Particle();
    Particle( int population_size );
    ~Particle();

    //particle filter functions
    /**
     * Initialise the model with random
     * number.
     */
    virtual void init_model() = 0;

    /**
     * Apply a random force to the particle, thus "predicting" the next position
     * of the particle. Also called motion model.
     */
    virtual void prediction() = 0;

    /**
     * Update the weight of the particle: compute the probability of
     * producing this measure given the current object position.
     *
     * @param measure A measure.
     */
    virtual void compute_probability( boost::shared_ptr<Measure> measure ) = 0;

    //accessors
    void set_weight( float weight );
    float get_weight();

protected:
    float weight;
    int population_size;

    /**
     * Initialise the weight of the particle to 1/population_size
     * @param population_size The size of the population.
     */
    void init_weight( int population_size );
};
}

#endif /* PARTICLE_HPP_ */
