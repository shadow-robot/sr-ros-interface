/**
 * @file   sr_movements.cpp
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
* You should have received a copy of the GNU General Public License alongpublish_rate
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief  Reads an image and publishes its content as std_msgs::Float64 on
 *         the targets topic.
 *
 *
 */


#include "sr_movements/movement_from_image.hpp"
#include "sr_movements/movement_publisher.hpp"

#include <ros/ros.h>
#include <iostream>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sr_movements");

  ros::NodeHandle nh_tilde("~");
  std::string img_path;

  if( nh_tilde.getParam("image_path", img_path) )
  {
    shadowrobot::MovementFromImage mvt_im( img_path );

    double min, max, publish_rate;
    if( !nh_tilde.getParam("publish_rate", publish_rate) )
      publish_rate = 100.0;

    int repetition;
    if( !nh_tilde.getParam("repetition", repetition) )
      repetition = 1;

    int nb_mvt_step;
    if( !nh_tilde.getParam("nb_step", nb_mvt_step) )
      nb_mvt_step = 1000;

    std::string controller_type;
    if( !nh_tilde.getParam("msg_type", controller_type) )
      controller_type = "";

    std::string joint_name;
    if( !nh_tilde.getParam("joint_name", joint_name) )
    {
      //for compatibility, if no joint_name is provided, then we use the
      // provided min and max to instantiate the movement publisher
      // which will use the remapped topics.
      if( !nh_tilde.getParam("min", min) )
        min = 0.0;
      if( !nh_tilde.getParam("max", max) )
        max = 1.5;

      shadowrobot::MovementPublisher mvt_pub( min, max, publish_rate,
                                              static_cast<unsigned int>(repetition),
                                              static_cast<unsigned int>(nb_mvt_step),
                                              controller_type);

      mvt_pub.add_movement( mvt_im );
      mvt_pub.start();
    }
    else
    {
      //check if this is a test using gazebo
      bool testing;
      if( !nh_tilde.getParam("testing", testing))
        testing = false;

      //This is the new easier to implement version of the movement
      // publisher. Simply uses the name of the joint to instantiate
      // the MovementPublisher, grabbing min, max and topics from the
      // HandCommander.
      shadowrobot::MovementPublisher mvt_pub( joint_name, publish_rate,
                                              static_cast<unsigned int>(repetition),
                                              static_cast<unsigned int>(nb_mvt_step),
                                              controller_type, testing);

      mvt_pub.add_movement( mvt_im );
      mvt_pub.start();
    }
  }
  else
  {
    ROS_ERROR("No bitmap specified");
  }
  return 0;
}



/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

