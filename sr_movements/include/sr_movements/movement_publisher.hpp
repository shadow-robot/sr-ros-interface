/**
 * @file   movement_publisher.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep 27 10:05:01 2011
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief  Publishes a sequence of movements.
 *
 */


#ifndef _MOVEMENT_PUBLISHER_HPP_
#define _MOVEMENT_PUBLISHER_HPP_

#include <ros/ros.h>
#include <boost/thread.hpp>

#include <std_msgs/Float64.h>
#include "sr_movements/partial_movement.hpp"

namespace shadowrobot
{
  class MovementPublisher
  {
  public:
    MovementPublisher( double min_value = 0.0, double max_value = 1.5,
                       double rate=100.0 );
    virtual ~MovementPublisher();

    void start();
    void stop();

    /**
     * Allows the user to control the movement step by
     * step.
     */
    void execute_step(int index_mvt_step, int index_partial_movement);

    void add_movement(PartialMovement mvt);

    void set_publisher(ros::Publisher publisher);

  protected:
    std::vector<PartialMovement> partial_movements;
    ros::NodeHandle nh_tilde;
    ros::Publisher pub;

    ros::Rate publishing_rate;
    double min, max;


    std_msgs::Float64 msg;
    double last_target_;

    static const unsigned int nb_mvt_step;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
