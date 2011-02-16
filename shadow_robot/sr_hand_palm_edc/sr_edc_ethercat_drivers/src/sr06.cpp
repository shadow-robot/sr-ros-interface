/*  This is a ROS driver for Shadow Robot #6 EtherCAT Slave
 *
 *  by Yann Sionneau <yann.sionneau@gmail.com>
 *  published under GPL
 */

#include <sr_edc_ethercat_drivers/sr06.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include <math.h>

using namespace std;

typedef unsigned char       int8u;
typedef   signed char       int8s;

typedef unsigned short      int16u;
typedef   signed short      int16s;

typedef unsigned int        int32u;
typedef   signed int        int32s;

extern "C" {
	#include "/home/fallen/Pic32/trunk/nodes/0220_palm_edc/0220_palm_edc_ethercat_protocol.h"
}

#define ETHERCAT_OUTGOING_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING)
#define ETHERCAT_INCOMING_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_INCOMING)

PLUGINLIB_REGISTER_CLASS(6, SR06, EthercatDevice);

SR06::SR06() : SR0X(), com_(EthercatDirectCom(EtherCAT_DataLinkLayer::instance()))
{
	counter_ = 0;
	lfj5_pub_ = nodehandle_.advertise<std_msgs::String>("lfj5", 1000);
}

SR06::~SR06()
{
	delete sh_->get_fmmu_config();
	delete sh_->get_pd_config();
}

void SR06::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
	SR0X::construct(sh, start_address);
	status_base_ = start_address;

	EC_FMMU *statusFMMU = new EC_FMMU(start_address,
					ETHERCAT_OUTGOING_DATA_SIZE,
					0x00, 
					0x07, 
					EC_PALM_EDC_DATA_PHY_BASE, 
					0x00, 
					true, 
					false, 
					true);

	start_address += ETHERCAT_OUTGOING_DATA_SIZE;
	command_base_ = start_address;

	EC_FMMU *commandFMMU = new EC_FMMU(start_address, 
					ETHERCAT_INCOMING_DATA_SIZE, 
					0x00, 
					0x07, 
					EC_PALM_EDC_COMMAND_PHY_BASE, 
					0x00, 
					false, 
					true, 
					true);

	start_address += ETHERCAT_INCOMING_DATA_SIZE;

	EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

	(*fmmu)[0] = *statusFMMU;
	(*fmmu)[1] = *commandFMMU;
	sh->set_fmmu_config(fmmu);

//	EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(0);
	EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

//	EC_SyncMan *commandSM = new EC_SyncMan(EC_PALM_EDC_COMMAND_PHY_BASE, ETHERCAT_INCOMING_DATA_SIZE, EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
//	commandSM->ChannelEnable = true;
//	commandSM->ALEventEnable = true;

//	EC_SyncMan *statusSM = new EC_SyncMan(EC_PALM_EDC_DATA_PHY_BASE, ETHERCAT_OUTGOING_DATA_SIZE);
//	statusSM->ChannelEnable = true;

	(*pd)[0] = EC_SyncMan(EC_PALM_EDC_DATA_PHY_BASE, ETHERCAT_OUTGOING_DATA_SIZE);
	(*pd)[1] = EC_SyncMan(EC_PALM_EDC_COMMAND_PHY_BASE, ETHERCAT_INCOMING_DATA_SIZE, EC_BUFFERED, EC_WRITTEN_FROM_MASTER);;

	(*pd)[0].ChannelEnable = true;

	(*pd)[1].ChannelEnable = true;
	(*pd)[1].ALEventEnable = true;

	sh->set_pd_config(pd);

	ROS_INFO("Finished to construct the SR06 driver");
}

int SR06::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{

  int retval = SR0X::initialize(hw, allow_unprogrammed);

  com_ = EthercatDirectCom(EtherCAT_DataLinkLayer::instance());

  return retval; 
}

void SR06::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *) {
stringstream name;
  name << "EtherCAT Device #" << setw(2) << setfill('0') 
       << sh_->get_ring_position() << " (Product SIX)";
  d.name = name.str();
  d.summary(d.OK, "OK");
  stringstream hwid;
  hwid << sh_->get_product_code() << "-" << sh_->get_serial();
  d.hardware_id = hwid.str();

  d.clear();
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product Code", "%d", sh_->get_product_code());
  d.addf("Serial Number", "%d", sh_->get_serial());
  d.addf("Revision", "%d", sh_->get_revision());
  d.addf("Counter", "%d", ++counter_);
  d.addf("LFJ5", "0x%04X", data_.LFJ5);
  EthercatDevice::ethercatDiagnostics(d, 2);
}

void SR06::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  //ROS_INFO("SR06::packCommand");
}

bool SR06::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
//  ROS_INFO("SR06::unpackState");
//  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING *tbuffer = (ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING *)(this_buffer + command_size_);
//  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING *pbuffer = (ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING *)(prev_buffer + command_size_);
//  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING data;

  readData(&com_, EC_PALM_EDC_DATA_PHY_BASE, &data_, ETHERCAT_OUTGOING_DATA_SIZE);

//  ROS_INFO("data : LFJ4 = 0x%04X ; LFJ5 = 0x%04X ; THJ1 = 0x%04X", data_.LFJ4, data_.LFJ5, data_.THJ1);

  std_msgs::String msg;
  std::stringstream ss;

  ss << data_.LFJ5;
  msg.data = ss.str();

  lfj5_pub_.publish(msg);

  return true;
}

