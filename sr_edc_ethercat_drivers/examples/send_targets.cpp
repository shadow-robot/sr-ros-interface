/**
 * @file   send_targets.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep  6 08:00:14 2011
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
 * @brief  This is an example which shows you how to send position targets to the
 * etherCAT hand.
 *
 */


#include <ros/ros.h>

//we store the publishers in a map.
#include <map>

//You need to send a Float64 message to the joints.
#include <std_msgs/Float64.h>

class TargetSender
{
public:
  /**
   * The TargetSender class stores a map of publishers to associate a
   * joint name to a publisher for easily sending targets to a given joint.
   *
   */
  TargetSender()
  {
    /**
     * This array of joints could also be guessed either using the pr2_controller
     * or listening to the joint_states topic.
     */
    static const std::string joints[20] = {"ffj0", "ffj3", "ffj4",
                                           "mfj0", "mfj3", "mfj4",
                                           "rfj0", "rfj3", "rfj4",
                                           "lfj0", "lfj3", "lfj4", "lfj5",
                                           "thj1", "thj2", "thj3", "thj4", "thj5",
                                           "wrj1", "wrj2"};

    static const std::string prefix = "/sh_";
    static const std::string suffix = "_position_controller/command";

    /*
     * We now build the map of publishers: there's one topic per joint.
     *  to send a target to a joint, we need to publish on /sh_ffj3_position_controller/command
     *  (replacing ffj3 by the other joint names, or position controller by the controller
     *  you want to use: you can run rostopic list to see which topics are available).
     *
     * The topic type is Float64.
     */
    for( unsigned int index_joint=0; index_joint < 20; ++index_joint )
    {
      std::string joint_tmp = joints[index_joint];
      publisher_map_[joint_tmp] = node_handle_.advertise<std_msgs::Float64>( prefix + joint_tmp + suffix , 2 );
    }
  };

  ~TargetSender()
  {};

  /**
   * Sends a target to the given joint.
   *
   * @param joint_name The joint we want to send the target to.
   * @param target The target in degrees.
   *
   * @return true if success.
   */
  bool sendupdate(std::string joint_name, double target)
  {
    //first we find the joint in the map (or return an error if we
    // can't find it)
    std::map<std::string, ros::Publisher>::iterator publisher_iterator;
    publisher_iterator = publisher_map_.find(joint_name);

    if( publisher_iterator == publisher_map_.end() )
    {
      ROS_WARN_STREAM("Joint "<< joint_name << " not found.");
      return false;
    }

    //now we build the message:
    // the target must be send in RADIANS
    std_msgs::Float64 msg_to_send;
    msg_to_send.data = target * 3.14159 / 360.0;
    publisher_iterator->second.publish( msg_to_send );

    return true;
  };

protected:
  /// The map were the publishers are stored.
  std::map<std::string, ros::Publisher> publisher_map_;
  /// A node handle to be able to publish.
  ros::NodeHandle node_handle_;
};


/**
 * The main instantiates a TargetSender and publishes
 * a target which grows continuously from 0 to 90, then
 * jumps back to 0, and starts again.
 *
 * The publishing stops when the node is killed (CTRL+C).
 *
 * @param argc
 * @param argv
 *
 * @return
 */
int main(int argc, char **argv)
{
  //Initialize the ROS node.
  ros::init (argc, argv, "send_targets");

  //Instantiate the target sender
  TargetSender target_sender = TargetSender();

  //Builds the vector of targets
  static const unsigned int length_targets = 1000;
  std::vector<double> targets;

  for( unsigned int i = 0; i < length_targets; ++i)
  {
    double target = static_cast<double>(i) / 1000.0 * 90.0;
    targets.push_back( target );
  }

  //Send the targets until the node is killed.
  unsigned int step = 0;
  while( ros::ok() )
  {
    target_sender.sendupdate("ffj3", targets[step % length_targets] );
    usleep(10000);

    ++ step;
  }

  return 0;
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
