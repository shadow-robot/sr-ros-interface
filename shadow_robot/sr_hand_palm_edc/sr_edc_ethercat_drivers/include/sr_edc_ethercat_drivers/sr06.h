/**
 * @file   sr06.h
 * @author Yann Sionneau <yann.sionneau@gmail.com>, Hugo Elias <hugo@shadowrobot.com>,
 *         Ugo Cupcic <ugo@shadowrobot.com>, contact <contact@shadowrobot.com>
 * @date   Mon May 23 13:33:30 2011
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
 * @brief This is a ROS driver for Shadow Robot #6 EtherCAT Slave
 *
 *
 */

#ifndef SR06_H
#define SR06_H

#include <ethercat_hardware/ethercat_device.h>
#include <sr_edc_ethercat_drivers/sr0x.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Int16.h>
#include <sr_edc_ethercat_drivers/SimpleMotorFlasher.h>
#include <pthread.h>
#include <bfd.h>
#include <boost/smart_ptr.hpp>
#include <map>
#include <boost/assign.hpp>

#include <sr_robot_lib/sr_hand_lib.hpp>

#include <sr_robot_msgs/TactileArray.h>
#include <sr_robot_msgs/Tactile.h>

#include <sr_external_dependencies/types_for_external.h>
extern "C"
{
  #include <sr_external_dependencies/external/0220_palm_edc/0220_palm_edc_ethercat_protocol.h>
}


class SR06 : public SR0X
{
public:
  SR06();
  ~SR06();

  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  int  initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);
  void multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer);

  bool simple_motor_flasher(sr_edc_ethercat_drivers::SimpleMotorFlasher::Request &req, sr_edc_ethercat_drivers::SimpleMotorFlasher::Response &res);
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
  bool can_data_is_ack(ETHERCAT_CAN_BRIDGE_DATA * packet);
  void erase_flash();
  bool read_flash(unsigned int offset, unsigned char baddrl, unsigned char baddrh, unsigned char baddru);

protected:
  int                                                                  counter_;
  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS                         data_;
  ros::NodeHandle                                                      nodehandle_;

  typedef realtime_tools::RealtimePublisher<std_msgs::Int16> rt_pub_int16_t;
  std::vector< boost::shared_ptr<rt_pub_int16_t> >   realtime_pub_;

private:

  static const unsigned int        nb_sensors_const;
  static const unsigned int        max_retry;
  static const unsigned short int  max_iter_const;
  static const unsigned short int  ros_pub_freq_const;
  static const unsigned short int  device_pub_freq_const;
  static const unsigned char       nb_publish_by_unpack_const;
  std::string                      firmware_file_name;
  pthread_mutex_t                  producing;
  ros::ServiceServer               serviceServer;

  bool                             flashing;
  ETHERCAT_CAN_BRIDGE_DATA         can_message_;
  bool                             can_message_sent;
  bool                             can_packet_acked;
  bfd_byte                        *binary_content; // buffer containing the binary content to be flashed
  unsigned int                     pos; // position in binary_content buffer
  unsigned int                     motor_being_flashed;

  ///counter for the number of empty buffer we're reading.
  unsigned int                     zero_buffer_read;

  boost::shared_ptr<shadow_robot::SrHandLib> sr_hand_lib;

  boost::shared_ptr<realtime_tools::RealtimePublisher<sr_robot_msgs::TactileArray> > tactile_publisher;
};


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif /* SR06_H */

