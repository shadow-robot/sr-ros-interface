/**
 * @file   3D_mouse.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Wed Jul 14 13:14:21 2010
 * 
 * @brief  
 * 
 * 
 */


//messages
#include <geometry_msgs/PoseStamped.h>

#include "threeD_mouse/3D_mouse.h"

namespace threedmouse
{

  const double ThreeDMouse::pi_over_360 = 0.0087266462599716477;

  ThreeDMouse::ThreeDMouse()
    : n_tilde("~"), publish_rate(0.0)
  {
    // set publish frequency
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 50.0);
    publish_rate = ros::Rate(publish_freq);

    //publishes JointState messages
    std::string prefix;
    std::string searched_param;
    n_tilde.searchParam("threedmouse_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "/pose";
    pub = node.advertise<geometry_msgs::PoseStamped>(full_topic, 2);

    if(!(dpy = XOpenDisplay(0))) 
      {
	ROS_FATAL( "failed to connect to the X server");
	ROS_BREAK();
      }

    bpix = BlackPixel(dpy, DefaultScreen(dpy));
    win = XCreateSimpleWindow(dpy, DefaultRootWindow(dpy), 0, 0, 1, 1, 0, bpix, bpix);

    /* This actually registers our window with the driver for receiving
     * motion/button events through the 3dxsrv-compatible X11 protocol.
     */
    if(spnav_x11_open(dpy, win) == -1) 
      {
	ROS_FATAL( "failed to connect to the space navigator daemon" );
	ROS_BREAK();
      }

    //set mouse mode to both by default
    mouse_mode = BOTH;
    //mouse mode changed => print info
    info_mouse_mode();

    thread_update_mouse_data = boost::thread(boost::bind( &ThreeDMouse::check_stopped, this ));
    thread_publish = boost::thread( boost::bind( &ThreeDMouse::update_mouse_data, this ));
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
    Quaternion quater;

    bool waiting_for_release = false;

    while(ros::ok())
      {
	spnav_poll_event(&sev);
	switch( sev.type )
	  {
	  case SPNAV_EVENT_MOTION:
	    last_pose.translation = Translation( sev.motion.x, sev.motion.y, sev.motion.z );

	    quater = euler_to_quaternion(sev.motion.rx, sev.motion.ry, sev.motion.rz);
	    last_pose.quaternion = Quaternion(quater);

	    ROS_DEBUG("got motion event: t(%d, %d, %d) ", sev.motion.x, sev.motion.y, sev.motion.z);
	    ROS_DEBUG("r(%d, %d, %d)", sev.motion.rx, sev.motion.ry, sev.motion.rz);

	    break;

	  case SPNAV_EVENT_BUTTON:
	    if(waiting_for_release)
	      {
		if(sev.button.press == false)
		  waiting_for_release = false;
	      }
	    else
	      {
		if(sev.button.press == true)
		  {
		    waiting_for_release = true;
		    if(sev.button.bnum == 0)
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
	  } // end switch spnav event type
	
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
    geometry_msgs::PoseStamped posestamped_msg;
    posestamped_msg.header.stamp = ros::Time::now();

    switch( mouse_mode )
      {
      case TRANSLATE:
	posestamped_msg.pose.position.x = last_pose.translation.x;
	posestamped_msg.pose.position.y = last_pose.translation.y;
	posestamped_msg.pose.position.z = last_pose.translation.z;
    
	posestamped_msg.pose.orientation.x = 0.0;
	posestamped_msg.pose.orientation.y = 0.0;
	posestamped_msg.pose.orientation.z = 0.0;
	posestamped_msg.pose.orientation.w = 0.0;
	break;

      case ROTATE:
    	posestamped_msg.pose.position.x = 0.0;
	posestamped_msg.pose.position.y = 0.0;
	posestamped_msg.pose.position.z = 0.0;
    
	posestamped_msg.pose.orientation.x = last_pose.quaternion.x;
	posestamped_msg.pose.orientation.y = last_pose.quaternion.y;
	posestamped_msg.pose.orientation.z = last_pose.quaternion.z;
	posestamped_msg.pose.orientation.w = last_pose.quaternion.w;
	break;

      default:
	posestamped_msg.pose.position.x = last_pose.translation.x;
	posestamped_msg.pose.position.y = last_pose.translation.y;
	posestamped_msg.pose.position.z = last_pose.translation.z;
    
	posestamped_msg.pose.orientation.x = last_pose.quaternion.x;
	posestamped_msg.pose.orientation.y = last_pose.quaternion.y;
	posestamped_msg.pose.orientation.z = last_pose.quaternion.z;
	posestamped_msg.pose.orientation.w = last_pose.quaternion.w;
	break;
      }

    pub.publish(posestamped_msg);
    
    ros::spinOnce();
    publish_rate.sleep();
  }

  Quaternion ThreeDMouse::euler_to_quaternion(float pitch, float yaw, float roll)
  {
    // Basically we create 3 Quaternions, one for pitch, one for yaw, one for roll
    // and multiply those together.
    // the calculation below does the same, just shorter

    Quaternion quater; 

    double p = pitch * pi_over_360;
    double y = yaw * pi_over_360;
    double r = roll * pi_over_360;
 
    double sinp = sin(p);
    double siny = sin(y);
    double sinr = sin(r);
    double cosp = cos(p);
    double cosy = cos(y);
    double cosr = cos(r);
 
    quater.x = sinr * cosp * cosy - cosr * sinp * siny;
    quater.y = cosr * sinp * cosy + sinr * cosp * siny;
    quater.z = cosr * cosp * siny - sinr * sinp * cosy;
    quater.w = cosr * cosp * cosy + sinr * sinp * siny;
 
    normalise(quater);

    return quater;
  }

  void ThreeDMouse::normalise(Quaternion quater)
  {
    // Don't normalize if we don't have to
    double mag2 = quater.w * quater.w + 
      quater.x * quater.x + quater.y * quater.y + 
      quater.z * quater.z;

    if (  mag2!=0.0 && (fabs(mag2 - 1.0f) > 0.01)) {
      double mag = sqrt(mag2);
      quater.w /= mag;
      quater.x /= mag;
      quater.y /= mag;
      quater.z /= mag;
    }
  }

}

