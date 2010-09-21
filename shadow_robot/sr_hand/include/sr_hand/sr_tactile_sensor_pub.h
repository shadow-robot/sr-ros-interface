/**
 * @file   sr_tactile_sensor_pub.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:26:41 2010
 * 
 * @brief  This class reads and publishes data concerning the
 * shadowhand. To publish those data, just call the publish()
 * function. 
 * 
 * 
 */

#ifndef SHADOWHAND_TACTILE_SENSOR_PUBLISHER_H_
#define SHADOWHAND_TACTILE_SENSOR_PUBLISHER_H_

#include <ros/ros.h>

using namespace ros;

namespace shadowrobot
{

class SRTactileSensorPublisher
{
public:
    /// Constructor
    SRTactileSensorPublisher();

    /// Destructor
    ~SRTactileSensorPublisher()
    {
    }
    ;

    void publish();

private:
    /////////////////
    //  CALLBACKS  //
    /////////////////
    //ros node handle
    NodeHandle node, n_tilde;
    Rate publish_rate;

    Publisher sr_jointstate_pub;
}; // end class ShadowhandTactileSensorPublisher

} // end namespace

#endif 	    /* !SHADOWHAND_TACTILE_SENSOR_PUBLISHER_H_ */
