/**
 * @file dataglove_processing.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#ifndef DATAGLOVE_PROCESSING_HPP_
#define DATAGLOVE_PROCESSING_HPP_

#include <ros/ros.h>
#include <vector>

#include "gtest/gtest_prod.h"
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include "threadpool/threadpool.hpp"

#include <math_utils.hpp>
#include "particle_sr_hand.hpp"
#include "measure.hpp"


namespace dataglove
{
struct CompareParticleWeights : std::binary_function<ParticleSrHand, ParticleSrHand, bool>
{
    CompareParticleWeights()
    {
    }
    bool operator()( const ParticleSrHand& v1, const ParticleSrHand& v2 ) const
    {
        return (v1.get_weight() < v2.get_weight());
    }
};

class DatagloveProcessing
{
    friend class DatagloveProcessingTest;
    FRIEND_TEST(DatagloveProcessingTest, resampling);
    FRIEND_TEST(DatagloveProcessingTest, pfInitWeights);
    FRIEND_TEST(DatagloveProcessingTest, pfInitPos);
    FRIEND_TEST(DatagloveProcessingTest, sortByWeights);
    FRIEND_TEST(DatagloveProcessingTest, updateCycle);

public:
    DatagloveProcessing();
    ~DatagloveProcessing();

    /**
     * This update loop will call update_cycle() which does the
     * actual update.
     *
     * @return 0 if ok, 1 if resampled, -1 if error
     */
    int update();

    //consts
    static const unsigned int total_number_of_particles;
    static const float average_weight;
    static const unsigned int n_min;

private:
    //ROS stuff
    ros::NodeHandle nh, nh_tilde;
    /// the rate at which the update loop of the particle filter is called
    ros::Rate update_rate;

    //threadpool
    boost::threadpool::pool threadpool;

    //type
    typedef boost::ptr_vector<ParticleSrHand>::iterator part_it_t;
    typedef std::auto_ptr< boost::ptr_vector<ParticleSrHand> > part_auto_ptr_t;

    //variables
    /// stores the particle cloud
    boost::shared_ptr< boost::ptr_vector<ParticleSrHand> > particle_cloud;
    /**
     *  needed when doing the resampling: temporary stores the
     *  new particle cloud, which is then retransfered to the
     *  particle_cloud variable.
     */
    boost::shared_ptr< boost::ptr_vector<ParticleSrHand> > particle_cloud_tmp;
    boost::shared_ptr<math_utils::MathUtils> math_utils;
    boost::shared_ptr<Measure> last_measure;
    //number of efficient particles (weight is big enough)
    unsigned int n_eff;
    //comparison structure
    CompareParticleWeights compare_particle_weights;
    ///sum_squared_weights needs to be thread safe
    float sum_squared_weights;
    boost::shared_ptr<boost::mutex> mutex_sum_squared_weights;

    //functions

    /**
     * Update each particle in a thread, using the threadpool.
     *
     * @return 0 if ok, 1 if resampled, -1 if error
     */
    int update_cycle();

    /**
     * The update loop will call each particle
     * update in a thread, using the threadpool.
     */
    void update_particle_in_thread();

    /**
     * Resample the point cloud using stochastic Universal Sampling(SUS).
     *
     * @return 1 if resampling, 0 if no resampling, -1 if error
     */
    int resampling();
    /**
     * Does a roulette wheel selection for a given fitness.
     * @param fitness
     * @return return a pointer to the selected particle
     */
    ParticleSrHand* roulette_wheel_selection( float fitness );

    std::vector<float> get_weights_vector();
    std::vector<std::vector<float> > get_particle_positions_vector();
}; // end class DatagloveProcessing

} //end namespace

#endif /* DATAGLOVE_PROCESSING_HPP_ */
