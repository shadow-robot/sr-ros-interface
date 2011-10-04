/**
 * @file   movement_publisher.cpp
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

#include "sr_movements/movement_publisher.hpp"

namespace shadowrobot
{
  const unsigned int MovementPublisher::nb_mvt_step = 1000;

  MovementPublisher::MovementPublisher(double min_value, double max_value,
                                       double rate, int repetition)
    : nh_tilde("~"), publishing_rate( rate ), repetition(repetition),
      min(min_value), max(max_value), last_target_(0.0)
  {
    pub = nh_tilde.advertise<std_msgs::Float64>("targets", 5);
  }

  MovementPublisher::~MovementPublisher()
  {}

  void MovementPublisher::start()
  {
    double last_target = 0.0;
    for(unsigned int i_rep = 0; i_rep < repetition; ++i_rep)
    {
      for( unsigned int i=0; i<partial_movements.size(); ++i)
      {
        for(unsigned int j=0; j<nb_mvt_step; ++j)
        {
          if( !ros::ok() )
            return;

          //get the target
          msg.data = partial_movements[i].get_target( static_cast<double>(j) / static_cast<double>(nb_mvt_step));
          //interpolate to the correct range
          msg.data = min + msg.data * (max - min);

          //there was not target -> resend the last target
          if( msg.data == -1.0 )
            msg.data = last_target;

          //publish the message
          pub.publish( msg );

          //wait for a bit
          publishing_rate.sleep();

          last_target = msg.data;
        }
      }
    }
  }


  void MovementPublisher::execute_step(int index_mvt_step, int index_partial_movement)
  {
    if( !ros::ok() )
      return;

    //get the target
    msg.data = partial_movements[index_partial_movement].get_target( static_cast<double>(index_mvt_step) / static_cast<double>(nb_mvt_step));
    //interpolate to the correct range
    msg.data = min + msg.data * (max - min);

    //there was not target -> resend the last target
    if( msg.data == -1.0 )
      msg.data = last_target_;

    //publish the message
    pub.publish( msg );

    //wait for a bit
    publishing_rate.sleep();

    last_target_ = msg.data;
  }

  void MovementPublisher::stop()
  {}

  void MovementPublisher::add_movement(PartialMovement mvt)
  {
    partial_movements.push_back( mvt );
  }

  void MovementPublisher::set_publisher(ros::Publisher publisher)
  {
    pub = publisher;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
