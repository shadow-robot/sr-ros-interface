#ifndef SR06_H
#define SR06_H

#include <ethercat_hardware/ethercat_device.h>
#include <sr_edc_ethercat_drivers/sr0x.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Int16.h>

typedef unsigned char       int8u;
typedef   signed char       int8s;

typedef unsigned short      int16u;
typedef   signed short      int16s;

typedef unsigned int        int32u;
typedef   signed int        int32s;

extern "C" {
	#include "/home/fallen/Pic32/trunk/nodes/0220_palm_edc/0220_palm_edc_ethercat_protocol.h"
}

class SR06 : public SR0X
{
public:
  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  int initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *);
  
  SR06();
  ~SR06();
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
protected:
  int counter_;
  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING data_;
  EthercatDirectCom com_;
  ros::NodeHandle nodehandle_;
  realtime_tools::RealtimePublisher<std_msgs::Int16> *realtime_pub_;
};

#endif /* SR06_H */

