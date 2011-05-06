/**
 * @file   cyberglove_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Apr 22 10:21:50 2010
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
 * @brief  The cyberglove node publishes data collected from a
 * Cyberglove.
 *
 *
 */

#include <ros/ros.h>
#include <time.h>
#include "cyberglove/cyberglove_publisher.h"
#include "cyberglove/cyberglove_service.h"
#include "cyberglove/Start.h"
#include <boost/smart_ptr.hpp>

using namespace cyberglove;

/////////////////////////////////
//           MAIN              //
/////////////////////////////////


/**
 *  Start the cyberglove publisher.
 *
 * @param argc
 * @param argv
 *
 * @return -1 if error (e.g. no glove found)
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cyberglove_publisher");
  //NodeHandle n;
  boost::shared_ptr<CyberglovePublisher> cyberglove_pub(new CyberglovePublisher());

  CybergloveService service(cyberglove_pub);

  ros::spin();

  return 0;
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
