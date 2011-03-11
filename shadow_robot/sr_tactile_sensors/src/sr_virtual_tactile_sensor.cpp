/**
 * @file   sr_virtual_tactile_sensor.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 10 11:07:10 2011
 * 
 * @brief  This is the virtual implementation of the SrGenericTactileSensor. It
 * computes virtual data.
 * 
 * 
 */

#include "sr_tactile_sensors/sr_virtual_tactile_sensor.hpp"

namespace shadowrobot
{
/**********************************
 *         TACTILE SENSOR         *
 **********************************/
  SrVirtualTactileSensor::SrVirtualTactileSensor(std::string name,
                                                 std::string touch_name,
                                                 std::string temp_name) :
    SrGenericTactileSensor(name, touch_name, temp_name)
  {
  }

  SrVirtualTactileSensor::~SrVirtualTactileSensor()
  {}

  double SrVirtualTactileSensor::get_touch_data()
  {
    return 0.0;
  }

  double SrVirtualTactileSensor::get_temp_data()
  {
    return 0.0;
  }


/**********************************
 *     TACTILE SENSOR MANAGER     *
 **********************************/
  SrVirtualTactileSensorManager::SrVirtualTactileSensorManager() :
    SrTactileSensorManager()
  {
    std::vector<std::vector<std::string> > all_names = get_all_names();

    for( unsigned int i=0; i<5; ++i)
    {
      tactile_sensors.push_back(
        boost::shared_ptr<SrVirtualTactileSensor>(
          new SrVirtualTactileSensor(all_names[0][i],
                                     all_names[1][i],
                                     all_names[2][i]) ));
    }
  }

  SrVirtualTactileSensorManager::~SrVirtualTactileSensorManager()
  {}
}


/** 
 * Initializes a set of virtual tactile sensors and publish.
 * 
 * @param argc 
 * @param argv 
 * 
 * @return -1 if error linking with the robot (i.e. robot code not started)
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sr_tactile_sensor");
  ros::NodeHandle n;

  shadowrobot::SrVirtualTactileSensorManager tact_sens_mgr;

  while( ros::ok() )
    tact_sens_mgr.publish_all();

  return 0;
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
