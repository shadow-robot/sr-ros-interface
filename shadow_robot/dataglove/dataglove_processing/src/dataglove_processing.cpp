/**
 * @file dataglove_processing.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#include "dataglove_processing.hpp"

namespace dataglove
{
const unsigned int DatagloveProcessing::total_number_of_particles = 100;
const float DatagloveProcessing::average_weight = 1.0f / ((float)DatagloveProcessing::total_number_of_particles);
const unsigned int DatagloveProcessing::n_min = (int)(DatagloveProcessing::total_number_of_particles * 0.7f);

DatagloveProcessing::DatagloveProcessing() :
    nh_tilde("~"), update_rate(0.0)
{
    math_utils = boost::shared_ptr<math_utils::MathUtils>(new math_utils::MathUtils());
    particle_cloud = boost::shared_ptr<boost::ptr_vector<ParticleSrHand> >(new boost::ptr_vector<ParticleSrHand>());
    particle_cloud_tmp = boost::shared_ptr<boost::ptr_vector<ParticleSrHand> >(new boost::ptr_vector<ParticleSrHand>());

    particle_cloud->reserve(total_number_of_particles);
    particle_cloud_tmp->reserve(total_number_of_particles);

    for( unsigned int particle_index = 0; particle_index < total_number_of_particles; ++particle_index )
    {
        particle_cloud->push_back(new ParticleSrHand(total_number_of_particles));
    }

    // set update frequency
    double update_freq;
    nh_tilde.param("update_frequency", update_freq, 20.0);
    update_rate = ros::Rate(update_freq);

    n_eff = 0;
}

DatagloveProcessing::~DatagloveProcessing()
{

}

void DatagloveProcessing::update()
{
    part_it_t particle;

    /*
     * n_eff measures the degenerecence of the cloud.
     * n_eff is computed during the update process
     * n_eff = 1/(sum(weights^2))
     */
    n_eff = 0;
    float sum_squared_weights = 0.0f;
    float tmp = 0.0f;
    for( particle = particle_cloud->begin(); particle != particle_cloud->end(); ++particle )
    {
        particle->prediction();
        tmp = particle->compute_probability(last_measure);
        sum_squared_weights += (tmp * tmp);
    }
    n_eff = 1.0f / sum_squared_weights;
    resampling();

    ros::spinOnce();
    update_rate.sleep();
}

int DatagloveProcessing::resampling()
{
    /*
     * don't resample if not enough convergence
     * n_eff is computed during the update process
     * n_eff = 1/(sum(weights^2))
     */
    if( n_eff > n_min )
        return -1;

    //reorder the vector by growing weights:
    particle_cloud->sort(compare_particle_weights);

    /*
     * The total fitness of the population has been normalized to 1.
     */
    float start = math_utils->maut_random(0.0f, average_weight);

    float fitness = 0.0f;
    for( float i = 0.0f; i < (float)total_number_of_particles; i = i + 1.0f )
    {
        fitness = start + i * average_weight;

        //run a roulette wheel selection for each value of fitness selected
        particle_cloud_tmp->push_back(roulette_wheel_selection(fitness));
    }
    particle_cloud->clear();

    /*
     * particle_cloud will now point to particle_cloud_tmp, and particle_cloud_tmp
     * is then reset.
     */
    particle_cloud = particle_cloud_tmp;
    particle_cloud_tmp = boost::shared_ptr<boost::ptr_vector<ParticleSrHand> >(new boost::ptr_vector<ParticleSrHand>());

    return 0;
}

ParticleSrHand* DatagloveProcessing::roulette_wheel_selection( float fitness )
{
    float tmp = 0.0f;
    float weight = 0.0f;
    part_auto_ptr_t part_tmp;

    for( part_it_t particle = particle_cloud->begin(); particle != particle_cloud->end(); ++particle )
    {
        weight = particle->get_weight();
        if( tmp < fitness && tmp + weight > fitness )
        {
            return new ParticleSrHand(particle, true, average_weight);
        }
        tmp += weight;
    }
    return NULL;
}

std::vector<float> DatagloveProcessing::get_weights_vector()
{
    std::vector<float> weights;
    part_it_t particle;
    for( particle = particle_cloud->begin(); particle != particle_cloud->end(); ++particle )
    {
        weights.push_back(particle->get_weight());
    }
    return weights;
}

std::vector<std::vector<float> > DatagloveProcessing::get_particle_positions_vector()
{
    std::vector<std::vector<float> > all_part_positions;
    part_it_t particle;
    for( particle = particle_cloud->begin(); particle != particle_cloud->end(); ++particle )
    {
        std::vector<float> part_pos;
        part_pos = particle->get_positions();
        all_part_positions.push_back(part_pos);
    }
    return all_part_positions;
}

}//end namespace
