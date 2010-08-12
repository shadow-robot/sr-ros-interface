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
#include "threeD_mouse/3D_mouse.h"

namespace threedmouse
{

  ThreeDMouse::ThreeDMouse()
    : n_tilde("~"), publish_rate(0.0)
  {
    // set publish frequency
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 50.0);
    publish_rate = ros::Rate(publish_freq);

    std::string searched_param;
    n_tilde.searchParam("published_tf_name", searched_param);
    n_tilde.param(searched_param, published_tf_name, std::string("threedmouse") );

    n_tilde.searchParam("reference_joint", searched_param);
    n_tilde.param(searched_param, reference_joint, std::string("sr_arm/position/shadowarm_handsupport"));


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
    bool waiting_for_release = false;

    while(ros::ok())
      {
	spnav_poll_event(&sev);
	switch( sev.type )
	  {
	  case SPNAV_EVENT_MOTION:
	    last_transform.setOrigin( tf::Vector3(sev.motion.x / 100.0, sev.motion.y / 100.0, sev.motion.z / 100.0 ));

	    
	    last_transform.setRotation( tf::Quaternion(sev.motion.rx / 57.3, sev.motion.ry / 57.3, sev.motion.rz / 57.3));

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
  

  //Need a mutex?
  void ThreeDMouse::publish()
  {
    switch( mouse_mode )
      {
      case TRANSLATE:
	last_transform.setRotation( tf::Quaternion(0.0, 0.0, 0.0) );
	break;

      case ROTATE:
    	last_transform.setOrigin( tf::Vector3(0.0,0.0,0.0) );
	break;

      default:
	break;
      }

    tf_broadcaster.sendTransform(tf::StampedTransform(last_transform, ros::Time::now(), reference_joint, published_tf_name)); 
    
    ros::spinOnce();
    publish_rate.sleep();
  }

}

