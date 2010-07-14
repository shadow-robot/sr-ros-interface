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
    //while(ros::ok())

    while(spnav_wait_event(&sev)) 
      {
	spnav_poll_event(&sev);
	if(sev.type == SPNAV_EVENT_MOTION) 
	  {
	    last_pose.x = sev.motion.x;
	    last_pose.y = sev.motion.y;
	    last_pose.z = sev.motion.z;

	    last_pose.rx = sev.motion.rx;
	    last_pose.ry = sev.motion.ry;
	    last_pose.rz = sev.motion.rz;
	    last_pose.w = 0.0;

	    ROS_DEBUG("got motion event: t(%d, %d, %d) ", sev.motion.x, sev.motion.y, sev.motion.z);
	    ROS_DEBUG("r(%d, %d, %d)", sev.motion.rx, sev.motion.ry, sev.motion.rz);
	  } 
	else //BUTTON PRESSED
	  {
	    if(sev.button.press == true)
	      {
		if(sev.button.bnum == 0)
		  mouse_mode == ROTATE ? mouse_mode = TRANSLATE : mouse_mode = ROTATE;
		else
		  mouse_mode == BOTH ? mouse_mode = TRANSLATE : mouse_mode = BOTH;
		
		//mouse mode changed => print info
		info_mouse_mode();
	      }
	  }
	
	if( !ros::ok() )
	  break;
	ros::spinOnce();
	//sleep(0.01);
      }
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
	posestamped_msg.pose.position.x = last_pose.x;
	posestamped_msg.pose.position.y = last_pose.y;
	posestamped_msg.pose.position.z = last_pose.z;
    
	posestamped_msg.pose.orientation.x = 0.0;
	posestamped_msg.pose.orientation.y = 0.0;
	posestamped_msg.pose.orientation.z = 0.0;
	posestamped_msg.pose.orientation.w = 0.0;
	break;

      case ROTATE:
    	posestamped_msg.pose.position.x = 0.0;
	posestamped_msg.pose.position.y = 0.0;
	posestamped_msg.pose.position.z = 0.0;
    

	posestamped_msg.pose.orientation.x = last_pose.rx;
	posestamped_msg.pose.orientation.y = last_pose.ry;
	posestamped_msg.pose.orientation.z = last_pose.rz;
	posestamped_msg.pose.orientation.w = last_pose.w;
	break;

      default:
	posestamped_msg.pose.position.x = last_pose.x;
	posestamped_msg.pose.position.y = last_pose.y;
	posestamped_msg.pose.position.z = last_pose.z;
    
	posestamped_msg.pose.orientation.x = last_pose.rx;
	posestamped_msg.pose.orientation.y = last_pose.ry;
	posestamped_msg.pose.orientation.z = last_pose.rz;
	posestamped_msg.pose.orientation.w = last_pose.w;
	break;
      }

    pub.publish(posestamped_msg);
    
    ros::spinOnce();
    publish_rate.sleep();
  }

}

