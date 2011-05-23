/**
 * @file   sr_diagnosticer.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 25 15:36:41 2010
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
 * @brief The goal of this ROS publisher is to publish relevant data
 * concerning the hand at a regular time interval.
 * Those data are (not exhaustive): positions, targets, temperatures,
 * currents, forces, error flags, ...
 *
 *
 */

//ROS include
#include <ros/ros.h>

//messages
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

//generic C/C++ include
#include <vector>
#include <string>
#include <sstream>

#include <boost/smart_ptr.hpp>

#include "sr_hand/sr_diagnosticer.h"

using namespace ros;
using namespace shadowrobot;

namespace shadowrobot
{

// 9 is the number of messages sent on the palm: used in
// converting rate to Hz
  const double SRDiagnosticer::palm_numb_msg_const = 9.0;
  const double SRDiagnosticer::palm_msg_rate_const = 4000.0;

/////////////////////////////////
//    CONSTRUCTOR/DESTRUCTOR   //
/////////////////////////////////

  SRDiagnosticer::SRDiagnosticer( boost::shared_ptr<SRArticulatedRobot> sr_art_robot, hardware_types hw_type ) :
    n_tilde("~"), publish_rate(0.0)
  {
    sr_articulated_robot = sr_art_robot;

    // set publish frequency
    double publish_freq;
    n_tilde.param("publish_frequency_diagnostics", publish_freq, 1.0);
    publish_rate = Rate(publish_freq);

    //publishes /diagnostics messages
    sr_diagnostics_pub = node.advertise<diagnostic_msgs::DiagnosticArray> ("diagnostics", 2);

    hardware_type = hw_type;
  }

  SRDiagnosticer::~SRDiagnosticer()
  {
    //if( shadowhand != NULL )
    // delete shadowhand;
  }

/////////////////////////////////
//       PUBLISH METHOD        //
/////////////////////////////////
  void SRDiagnosticer::publish()
  {
    diagnostic_msgs::DiagnosticArray diag_msg;

    std::vector<diagnostic_msgs::DiagnosticStatus> vec_diag_msg;

    std::vector<DiagnosticData> diagnostics = sr_articulated_robot->getDiagnostics();

    std::stringstream ss;

    for( unsigned int i = 0; i < diagnostics.size(); ++i )
    {
      diagnostic_msgs::DiagnosticStatus diag;

      std::vector<diagnostic_msgs::KeyValue> keyvalues;

      diag.level = diagnostics[i].level;

      switch( hardware_type )
      {
      case sr_hand_hardware:
        diag.name = "srh/" + diagnostics[i].joint_name;
        break;
      case sr_arm_hardware:
        diag.name = "sr_arm/" + diagnostics[i].joint_name;
        break;
      default:
        diag.name = diagnostics[i].joint_name;
        break;
      }

      diagnostic_msgs::KeyValue keyval;
      keyval.key = "Target";

      ss.str("");
      ss << diagnostics[i].target;
      keyval.value = ss.str();
      keyvalues.push_back(keyval);

      keyval.key = "Position";
      ss.str("");
      ss << diagnostics[i].position;
      keyval.value = ss.str();
      keyvalues.push_back(keyval);

      keyval.key = "Flags";
      ss.str("");
      ss << diagnostics[i].flags;
      keyval.value = ss.str();
      keyvalues.push_back(keyval);

      //get all the debug values
      std::map<const std::string, const unsigned int>::const_iterator iter;
      for(iter = debug_values::names_and_offsets.begin();
          iter !=  debug_values::names_and_offsets.end(); ++iter)
      {
        keyval.key = iter->first;
        ss.str("");
        ss << diagnostics[i].debug_values[iter->first];
        keyval.value = ss.str();
        keyvalues.push_back(keyval);
      }
      if( diag.level == 0 )
        diag.message = "OK";

      diag.values = keyvalues;
      vec_diag_msg.push_back(diag);
    }

    //set the standard message
    diag_msg.status = vec_diag_msg;
    //publish the diagnostic data

    diag_msg.header.stamp = ros::Time::now();
    sr_diagnostics_pub.publish(diag_msg);

    ros::spinOnce();
    publish_rate.sleep();
  }

}// end namespace


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

