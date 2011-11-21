/**
 * @file   3D_mouse.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jul 14 13:14:21 2010
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
 * @brief
 *
 *
 */

//messages
#include "threeD_mouse/3D_mouse.h"

namespace threedmouse
{
  const double ThreeDMouse::translation_dampening = 4000.0;
  const double ThreeDMouse::rotation_dampening = 10.0*57.0;

  ThreeDMouse::ThreeDMouse() :
    n_tilde("~"), publish_rate(0.0)
  {
    // set publish frequency
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 50.0);
    publish_rate = ros::Rate(publish_freq);

    std::string searched_param;
    n_tilde.searchParam("published_tf_name", searched_param);
    n_tilde.param(searched_param, published_tf_name, std::string("threedmouse"));

    n_tilde.searchParam("reference_joint", searched_param);
    n_tilde.param(searched_param, reference_joint, std::string("sr_arm/position/shadowarm_handsupport_motor"));

    if( !(dpy = XOpenDisplay(0)) )
    {
      ROS_FATAL( "failed to connect to the X server");
      ROS_BREAK();
    }

    bpix = BlackPixel(dpy, DefaultScreen(dpy));
    win = XCreateSimpleWindow(dpy, DefaultRootWindow(dpy), 0, 0, 1, 1, 0, bpix, bpix);

    /* This actually registers our window with the driver for receiving
     * motion/button events through the 3dxsrv-compatible X11 protocol.
     */
    if( spnav_x11_open(dpy, win) == -1 )
    {
      ROS_FATAL( "failed to connect to the space navigator daemon" );
      ROS_BREAK();
    }

    //set mouse mode to both by default
    mouse_mode = BOTH;
    //mouse mode changed => print info
    info_mouse_mode();

    thread_publish = boost::thread(boost::bind(&ThreeDMouse::update_mouse_data, this));
    thread_update_mouse_data = boost::thread(boost::bind(&ThreeDMouse::check_stopped, this));
  }

  ThreeDMouse::~ThreeDMouse()
  {
    thread_update_mouse_data.join();
    thread_publish.join();

    spnav_close();
  }

  void ThreeDMouse::check_stopped()
  {
  }

  void ThreeDMouse::update_mouse_data()
  {
    bool waiting_for_release = false;

    while( ros::ok() )
    {
      spnav_poll_event(&sev);
      switch( sev.type )
      {
      case SPNAV_EVENT_MOTION:
        if( !mutex_last_transform.try_lock() )
          continue;

        ROS_DEBUG("3D mouse: [%f, %f, %f]", (double)sev.motion.x / translation_dampening, (double)sev.motion.y / translation_dampening, (double)sev.motion.z / translation_dampening);
        last_transform.setOrigin(tf::Vector3(-(double)sev.motion.x / translation_dampening, (double)sev.motion.y / translation_dampening, (double)sev.motion.z / translation_dampening));
        ROS_DEBUG("3D mouse: [%f, %f, %f]", (double)sev.motion.rx / rotation_dampening, (double)sev.motion.ry / rotation_dampening, (double)sev.motion.rz / rotation_dampening);
        last_transform.setRotation(tf::Quaternion((double)sev.motion.rx / rotation_dampening, (double)sev.motion.ry / rotation_dampening, (double)sev.motion.rz / rotation_dampening));

        mutex_last_transform.unlock();
        break;

      case SPNAV_EVENT_BUTTON:
        if( waiting_for_release )
        {
          if( sev.button.press == false )
            waiting_for_release = false;
        }
        else
        {
          if( sev.button.press == true )
          {
            waiting_for_release = true;
            if( sev.button.bnum == 0 )
              mouse_mode == ROTATE ? mouse_mode = TRANSLATE : mouse_mode = ROTATE;
            else
              mouse_mode == BOTH ? mouse_mode = TRANSLATE : mouse_mode = BOTH;

            //mouse mode changed => print info
            info_mouse_mode();
          }
        }
        break;

      default:
        break;
      }

      ros::spinOnce();
      usleep(1000);
    } //end while
  }

/**
 * Print some relevant information about the current mouse mode.
 *
 */
  void ThreeDMouse::info_mouse_mode()
  {
    switch( mouse_mode )
    {
    case TRANSLATE:
      ROS_INFO("3d mouse set to translate mode");
      break;
    case ROTATE:
      ROS_INFO("3d mouse set to rotate mode");
      break;
    case BOTH:
      ROS_INFO("3d mouse set to both mode");
      break;
    default:
      ROS_WARN("3d mouse mode not recognized");
      break;
    }
  }

  void ThreeDMouse::publish()
  {
    mutex_last_transform.lock();

    switch( mouse_mode )
    {
    case TRANSLATE:
      last_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0));
      break;

    case ROTATE:
      last_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
      break;

    default:
      break;
    }

    tf_broadcaster.sendTransform(tf::StampedTransform(last_transform, ros::Time::now(), reference_joint, published_tf_name));

    mutex_last_transform.unlock();

    ros::spinOnce();
    publish_rate.sleep();
  }

}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
