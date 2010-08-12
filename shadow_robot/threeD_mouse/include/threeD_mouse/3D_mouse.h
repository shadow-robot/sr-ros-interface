/**
 * @file   3D_mouse.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jul 14 10:15:29 2010
 * 
 * @brief  
 * 
 * 
 */

#ifndef _3D_MOUSE_H_
#define _3D_MOUSE_H_

#include <ros/ros.h>

#include <X11/Xlib.h>
#include <spnav.h>
#include <boost/thread.hpp>

#include <tf/transform_broadcaster.h>


namespace threedmouse 
{
  /**
   *enum to define the mode we're in
   **/
  enum MouseMode 
  {
    ROTATE,
    TRANSLATE,
    BOTH
  };

  class ThreeDMouse
  {
  public:
    ThreeDMouse();
    ~ThreeDMouse();

    void publish();
  private:
    ///ros node handle
    ros::NodeHandle node, n_tilde;
    ///the rate at which the data will be published. This can be set by a parameter in the launch file.
    ros::Rate publish_rate;

    ///the name to assign to the published transform
    std::string published_tf_name;
    /// the reference joint (e.g. the tip of the arm) from which the transform is computed.
    std::string reference_joint;

    ///The transform broadcaster
    tf::TransformBroadcaster tf_broadcaster;

    void update_mouse_data();

    boost::thread thread_update_mouse_data;
    boost::thread thread_publish;

    //for the 3d mouse
    spnav_event sev;
    Display *dpy;
    Window win;
    unsigned long bpix;

    //keep the last pose read from the 3d mouse
    tf::Transform last_transform;

    MouseMode mouse_mode;

    void info_mouse_mode();
    void check_stopped();

    boost::mutex mutex_last_transform;
  };
}


#endif
