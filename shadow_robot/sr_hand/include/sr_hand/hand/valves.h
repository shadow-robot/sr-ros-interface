/**
 * @file   valves.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Feb 9 13:56:10 2010
 *
 * @brief This is a ROS Interface used to directly access the valves 
 * on Shadow Robot's muscle robots.
 *
 */

#ifndef VALVES_H_
#define VALVES_H_

#include <robot/config.h>
#include <robot/robot.h>
#include <robot/hand.h>
#include <robot/hand_protocol.h>
#include <ros/ros.h>

#include <std_msgs/Float64.h>

namespace shadowrobot
{

  class Valves
  {
  public:
    /**
     * Initializes the necessary mappings with a static list of names.
     */
    Valves();
    ///destructor
    ~Valves();

    void publish();

  private:
    void valve_command(const std_msgs::Float64ConstPtr& msg, int index_valve);

    std::vector<struct sensor> valves_sensors;

    std::vector<ros::Publisher> valves_publishers;
    std::vector<ros::Subscriber> valves_subscribers;

    ros::NodeHandle n_tilde;
    ros::Rate publish_rate;

    void init_subs_and_pubs(int index_joint);
  };

}//end namespace

#endif /* VALVES_H_ */

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

