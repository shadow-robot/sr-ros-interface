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

namespace threedmouse 
{
  struct Quaternion
  {

    double x;
    double y;
    double z;
    double w;

  Quaternion() :
    x(0.0), y(0.0), z(0.0), w(0.0)
    {}

  Quaternion(double nx, double ny, double nz, double nw) :
    x(nx), y(ny), z(nz), w(nw)
    {}

  Quaternion(Quaternion& q) :
    x(q.x), y(q.y), z(q.z), w(q.w)
    {}

  Quaternion(const Quaternion& q) :
    x(q.x), y(q.y), z(q.z), w(q.w)
    {}

  };

  struct Translation
  {
    double x;
    double y;
    double z;

  Translation() :
    x(0.0), y(0.0), z(0.0)
    {}

  Translation(double nx, double ny, double nz) : 
    x(nx), y(ny), z(nz)
    {
    }

  Translation(Translation& t) : 
    x(t.x), y(t.y), z(t.z)
    {
    }

  Translation(const Translation& t) : 
    x(t.x), y(t.y), z(t.z)
    {
    }
  };

  struct Pose
  {
    Translation translation;

    Quaternion quaternion;
  };

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
    ///The publisher which publishes the data to the \/{prefix}\/joint_states topic.
    ros::Publisher pub;

    void update_mouse_data();

    boost::thread thread_update_mouse_data;
    boost::thread thread_publish;

    //for the 3d mouse
    spnav_event sev;
    Display *dpy;
    Window win;
    unsigned long bpix;

    //keep the last pose read from the 3d mouse
    Pose last_pose;

    MouseMode mouse_mode;

    void info_mouse_mode();
    void check_stopped();

    // Convert roll, pitch, yaw (Euler Angles) to Quaternion
    Quaternion euler_to_quaternion(float pitch, float yaw, float roll);
    void normalise(Quaternion quater);
  };
}


#endif
