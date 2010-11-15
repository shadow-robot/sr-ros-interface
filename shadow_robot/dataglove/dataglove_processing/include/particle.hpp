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
     */
    virtual void update() = 0;

    //accessors
    float get_weight() const;
    void set_weight( float new_weight );
    void set_last_measure( boost::shared_ptr<Measure> last_measure );

protected:
    float weight;
    float squared_weight;
    int population_size;
    boost::shared_ptr<Measure> last_measure;

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
     * @return the new weight
     */
    virtual float compute_probability( ) = 0;

};
}

#endif /* PARTICLE_HPP_ */
