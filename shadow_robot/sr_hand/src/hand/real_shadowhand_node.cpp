/**
 * @file   real_shadowhand_node.cpp
 * @author Ugo Cupcic <ugo@ugo-kubuntu.local>
 * @date   Wed Apr  7 15:37:06 2010
 *
 * @brief Contains the main for the real Shadow Dextrous Hand. We start the publishers / subscribers in this node.
 * They all share the same RealShadowhand object, this way the subscriber can update the hand properties,
 * while the publishers publish up to date data. The diagnostics and the other publisher are started
 * in two different threads, to allow them to be published at different frequencies.
 *
 *
 */

#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <kdl/tree.hpp>

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "sr_hand/sr_subscriber.h"
#include "sr_hand/sr_publisher.h"
#include "sr_hand/sr_diagnosticer.h"
//#include "shadowhand/shadowhand_config_server.h"
#include "sr_hand/hand/real_shadowhand.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace shadowhand_subscriber;
using namespace shadowhand_publisher;
using namespace shadowhand_diagnosticer;
using namespace shadowhand;
//using namespace shadowhand_config_server;

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////

void run_diagnotics(boost::shared_ptr<ShadowhandDiagnosticer> shadowhand_diag)
{
  while( ok() )
  {
    shadowhand_diag->publish();
  }
}

void run_publisher(boost::shared_ptr<ShadowhandPublisher> shadowhand_pub)
{
  while( ok() )
  {
    shadowhand_pub->publish();
  }
}

/** 
 * The main function initializes the links with the robot, initializes
 * this ROS subscriber and sets the different callbacks.
 * This ROS subscriber will listen for new commands and send them to
 * the robot.
 * 
 * @param argc 
 * @param argv 
 * 
 * @return -1 if error linking with the robot (i.e. robot code not started)
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "shadowhand");
  NodeHandle n;

  boost::shared_ptr<RealShadowhand> real_sh(new RealShadowhand());
  boost::shared_ptr<ShadowhandSubscriber> shadowhand_subscriber;


  boost::shared_ptr<ShadowhandPublisher> shadowhand_pub( new ShadowhandPublisher(real_sh));
  boost::shared_ptr<ShadowhandDiagnosticer> shadowhand_diag( new ShadowhandDiagnosticer(real_sh, sr_hand_hardware));

  // gets the location of the robot description on the parameter server
  string full_param_name;
  n.searchParam("robot_description",full_param_name);

  string robot_desc_string;
  n.param(full_param_name, robot_desc_string, string());
  Tree tree;
  if (!kdl_parser::treeFromString(robot_desc_string, tree))
    {
      ROS_ERROR("Failed to construct kdl tree");
    }
  else
    {
      ROS_DEBUG("kdl tree loaded!");
    }
  
  //  ShadowhandConfigServer shadowhand_config_server;
  if (tree.getNrOfSegments() == 0)
    {
      ROS_WARN("ShadowHand subscriber got an empty tree and cannot do inverse kinematics");
      shadowhand_subscriber = boost::shared_ptr<ShadowhandSubscriber>(new ShadowhandSubscriber(real_sh));
    }
  else if (tree.getNrOfSegments() == 1)
    {
      ROS_WARN("ShadowHand subscriber got an empty tree and cannot do inverse kinematics");

      shadowhand_subscriber = boost::shared_ptr<ShadowhandSubscriber>(new ShadowhandSubscriber(real_sh));
    }
  else
    {
      shadowhand_subscriber = boost::shared_ptr<ShadowhandSubscriber>(new  ShadowhandSubscriber(real_sh, tree));
    }
    
  boost::thread thrd1( boost::bind( &run_diagnotics, shadowhand_diag ));
  boost::thread thrd2( boost::bind( &run_publisher, shadowhand_pub ));
  thrd1.join();
  thrd2.join();
  
  return 0;
}
