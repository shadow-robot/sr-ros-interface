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

#include <boost/ptr_container/ptr_vector.hpp>
#include <particle_sr_hand.hpp>

namespace dataglove
{
class DatagloveProcessing
{
public:
    DatagloveProcessing();
    ~DatagloveProcessing();

    void update();

private:
    //consts
    static const unsigned int total_number_of_particles;

    //variables
    boost::ptr_vector<ParticleSrHand> particle_cloud;

};
}

#endif /* DATAGLOVE_PROCESSING_HPP_ */
