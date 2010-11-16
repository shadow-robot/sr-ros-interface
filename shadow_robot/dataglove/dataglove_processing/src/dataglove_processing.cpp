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
const unsigned int DatagloveProcessing::total_number_of_particles = 200;
const float DatagloveProcessing::average_weight = 1.0f / ((float)DatagloveProcessing::total_number_of_particles);
const float DatagloveProcessing::n_min = (DatagloveProcessing::total_number_of_particles * 0.7f);

DatagloveProcessing::DatagloveProcessing() :
    nh_tilde("~"), update_rate(0.0)
{
    //detect the optimal number of threads to use in the thread pool
    int nb_possible_threads = boost::thread::hardware_concurrency();
    ROS_INFO("will use %d threads", nb_possible_threads);
    threadpool = boost::threadpool::pool(nb_possible_threads);

    math_utils = boost::shared_ptr<math_utils::MathUtils>(new math_utils::MathUtils());
    particle_cloud = boost::shared_ptr<boost::ptr_vector<ParticleSrHand> >(new boost::ptr_vector<ParticleSrHand>());
    particle_cloud_tmp = boost::shared_ptr<boost::ptr_vector<ParticleSrHand> >(new boost::ptr_vector<ParticleSrHand>());

    particle_cloud->reserve(total_number_of_particles);
    particle_cloud_tmp->reserve(total_number_of_particles);

    n_eff_standard = 0.0f;
    for( unsigned int particle_index = 0; particle_index < total_number_of_particles; ++particle_index )
    {
        particle_cloud->push_back(new ParticleSrHand(total_number_of_particles));
        n_eff_standard += (average_weight * average_weight);
    }

    n_eff_standard = 1.0f / n_eff_standard;

    mutex_sum_weights = boost::shared_ptr<boost::mutex>(new boost::mutex());
    sum_weights = boost::shared_ptr<float>(new float(0.0f));
    sum_squared_weights = boost::shared_ptr<float>(new float(0.0f));

    // set update frequency
    double update_freq;
    nh_tilde.param("update_frequency", update_freq, 20.0);
    update_rate = ros::Rate(update_freq);

    n_eff = n_eff_standard;
}

DatagloveProcessing::~DatagloveProcessing()
{

}

int DatagloveProcessing::update()
{
    int result = update_cycle();

    ros::spinOnce();
    update_rate.sleep();

    return result;
}

int DatagloveProcessing::update_cycle()
{
    part_it_t particle;

    /*
     * n_eff measures the degenerecence of the cloud.
     * n_eff is computed during the update process
     * n_eff = 1/(sum(weights^2))
     */
    n_eff = 0;
    for( particle = particle_cloud->begin(); particle != particle_cloud->end(); ++particle )
    {
        particle->set_last_measure(last_measure);

        threadpool.schedule(boost::bind(&ParticleSrHand::update, &*particle, sum_weights.get(), mutex_sum_weights.get()));
        //sum_squared_weights += (tmp * tmp);
    }
    //wait until all the tasks are finished
    threadpool.wait();

    //Normalise the weights + set square weights at the same time.
    for( particle = particle_cloud->begin(); particle != particle_cloud->end(); ++particle )
    {
        *sum_squared_weights += particle->set_weight(particle->get_weight() / (*sum_weights));
    }

    //recompute the n_eff (used to check if resampling is necessary or not)
    n_eff = 1.0f / *sum_squared_weights;

    //resample if necessary
    return resampling();
}

int DatagloveProcessing::resampling()
{
    /*
     * don't resample if not enough convergence
     * n_eff is computed during the update process
     * n_eff = 1/(sum(weights^2))
     */
    if( n_eff > n_min )
        return 0;

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
     * particle_cloud will now point to particle_cloud_tmp and particle_cloud_tmp is
     * then reset.
     */
    particle_cloud = particle_cloud_tmp;
    particle_cloud_tmp = boost::shared_ptr<boost::ptr_vector<ParticleSrHand> >(new boost::ptr_vector<ParticleSrHand>());

    n_eff = n_eff_standard;
    return 1;
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
