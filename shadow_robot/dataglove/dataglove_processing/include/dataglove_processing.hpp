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

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/smart_ptr.hpp>

#include "particle_sr_hand.hpp"
#include "measure.hpp"


namespace dataglove
{
class DatagloveProcessing
{
public:
    DatagloveProcessing();
    ~DatagloveProcessing();

    void update();

    //accessors
    unsigned int get_total_number_of_particles();
    std::vector<float> get_weights_vector();

private:
    ros::NodeHandle nh, nh_tilde;
    /// the rate at which the update loop of the particle filter is called
    ros::Rate update_rate;
    //consts
    static const unsigned int total_number_of_particles;

    //variables
    boost::ptr_vector<ParticleSrHand> particle_cloud;
    boost::shared_ptr<Measure> last_measure;

    //functions
    void resampling();

};
}

#endif /* DATAGLOVE_PROCESSING_HPP_ */
