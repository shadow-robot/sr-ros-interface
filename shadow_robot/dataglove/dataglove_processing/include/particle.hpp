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
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace dataglove
{
class Particle : boost::noncopyable
{
public:
    Particle();
    /**
     * initialises the weight to 1/population_size
     * @param population_size the size of the cloud
     */
    Particle( int population_size );
    ///copy constructor
    Particle( boost::ptr_vector<Particle>::iterator particle, bool reset_weight, float average_weight );
    ~Particle();

    //particle filter functions
    /**
     * Initialise the model with random
     * number.
     */
    virtual void init_model() = 0;

    /**
     * Runs the update cycle: first prediction()
     * then compute_probability(last_measure)
     *
     * @param sum_squared_weights a pointer to the Sum of weights for the whole cloud
     * @param mutex_sum_squared_weights a pointer to the mutex used to block this variable
     */
    virtual void update( float* sum_weights, boost::mutex* mutex_sum_weights, boost::shared_ptr<std::vector<boost::shared_ptr<Measure> > > last_measures_for_processing ) = 0;

    //accessors
    float get_weight() const;

    /**
     * set the weight / squared weight for the current particle
     * @param new_weight the new weight to set
     * @return the squared weight value
     */
    float set_weight( float new_weight );

protected:
    float weight;
    float squared_weight;
    int population_size;

    /**
     * Initialise the weight of the particle to 1/population_size
     * @param population_size The size of the population.
     */
    void init_weight( int population_size );

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
    virtual void compute_probability( boost::shared_ptr<std::vector<boost::shared_ptr<Measure> > > last_measures_for_processing ) = 0;

};
}

#endif /* PARTICLE_HPP_ */
