/**
 * @file dataglove_processing_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date 9 Nov 2010
 *
 * @brief 
 *
 *
 */

#include "dataglove_processing.hpp"

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

using namespace dataglove;

void update( boost::shared_ptr<DatagloveProcessing> dataglove_processing )
{
    while( ros::ok() )
        dataglove_processing->update();
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "dataglove_processing");
    ros::NodeHandle n;

    boost::shared_ptr<DatagloveProcessing> dataglove_processing(new DatagloveProcessing());

    boost::thread thread_update(boost::bind(&update, dataglove_processing));
    thread_update.join();
    return 0;
}
