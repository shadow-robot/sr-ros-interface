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
#include <realtime_tools/realtime_publisher.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include <std_msgs/Int16.h>
#include <math.h>

using namespace std;

typedef unsigned char       int8u;
typedef   signed char       int8s;

typedef unsigned short      int16u;
typedef   signed short      int16s;

typedef unsigned int        int32u;
typedef   signed int        int32s;

extern "C" {
	#include "/home/hand/0220_palm_edc_ethercat_protocol.h"
}

const unsigned short int SR06::device_pub_freq_const = 1000;
const unsigned short int SR06::ros_pub_freq_const = 100;
const unsigned short int SR06::max_iter_const = device_pub_freq_const / ros_pub_freq_const;
const unsigned char SR06::nb_sensors_const = 36;
const unsigned char SR06::nb_publish_by_unpack_const = (nb_sensors_const % max_iter_const) ? (nb_sensors_const / max_iter_const) + 1 : (nb_sensors_const / max_iter_const);

#define ETHERCAT_OUTGOING_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING)
#define ETHERCAT_INCOMING_DATA_SIZE sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_INCOMING)

#define ETHERCAT_CAN_BRIDGE_DATA_SIZE sizeof(ETHERCAT_CAN_BRIDGE_DATA)

PLUGINLIB_REGISTER_CLASS(6, SR06, EthercatDevice);

SR06::SR06() : SR0X()//, com_(EthercatDirectCom(EtherCAT_DataLinkLayer::instance()))
{
	char topic_name[4];
	unsigned char i;
	counter_ = 0;
	for (i = 0 ; i < nb_sensors_const ; ++i) {
		sprintf(topic_name, "j%d", i);
		realtime_pub_.push_back(new realtime_tools::RealtimePublisher<std_msgs::Int16>(nodehandle_, topic_name, 1000));
	}
	ROS_INFO("device_pub_freq_const = %d", device_pub_freq_const);
	ROS_INFO("ros_pub_freq_const = %d", ros_pub_freq_const);
	ROS_INFO("max_iter_const = %d", max_iter_const);
	ROS_INFO("nb_sensors_const = %d", nb_sensors_const);
	ROS_INFO("nb_publish_by_unpack_const = %d", nb_publish_by_unpack_const);
}

SR06::~SR06()
{
	delete sh_->get_fmmu_config();
	delete sh_->get_pd_config();
}

void SR06::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
	SR0X::construct(sh, start_address);

	command_base_ = start_address;
	command_size_ = ETHERCAT_INCOMING_DATA_SIZE + ETHERCAT_CAN_BRIDGE_DATA_SIZE;
	ROS_ERROR("First FMMU (command) : start_address : 0x%08X ; size : 0x%08X ; phy addr : 0x%08X\n", start_address, ETHERCAT_INCOMING_DATA_SIZE, EC_PALM_EDC_COMMAND_PHY_BASE);
	EC_FMMU *commandFMMU = new EC_FMMU(start_address, 
					ETHERCAT_INCOMING_DATA_SIZE + ETHERCAT_CAN_BRIDGE_DATA_SIZE, 
					0x00, 
					0x07, 
					EC_PALM_EDC_COMMAND_PHY_BASE, 
					0x00, 
					false, 
					true, 
					true);
	start_address += ETHERCAT_INCOMING_DATA_SIZE;
	start_address += ETHERCAT_CAN_BRIDGE_DATA_SIZE;

	status_base_ = start_address;
	
	ROS_ERROR("Second FMMU (status) : start_address : 0x%08X ; size : 0x%08X ; phy addr : 0x%08X\n", start_address, ETHERCAT_OUTGOING_DATA_SIZE, EC_PALM_EDC_DATA_PHY_BASE);

	EC_FMMU *statusFMMU = new EC_FMMU(start_address,
					ETHERCAT_OUTGOING_DATA_SIZE + ETHERCAT_CAN_BRIDGE_DATA_SIZE,
					0x00, 
					0x07, 
					EC_PALM_EDC_DATA_PHY_BASE, 
					0x00, 
					true, 
					false, 
					true);
	
	status_size_ = ETHERCAT_OUTGOING_DATA_SIZE + ETHERCAT_CAN_BRIDGE_DATA_SIZE; 

	EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

	(*fmmu)[0] = *commandFMMU;
	(*fmmu)[1] = *statusFMMU;
	
	sh->set_fmmu_config(fmmu);

	EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(4);

	(*pd)[0] = EC_SyncMan(EC_PALM_EDC_COMMAND_PHY_BASE, ETHERCAT_INCOMING_DATA_SIZE, EC_QUEUED, EC_WRITTEN_FROM_MASTER);
	(*pd)[1] = EC_SyncMan(EC_PALM_EDC_CAN_BRIDGE_MASTER_OUT_BASE, ETHERCAT_CAN_BRIDGE_DATA_SIZE, EC_QUEUED, EC_WRITTEN_FROM_MASTER);
	(*pd)[2] = EC_SyncMan(EC_PALM_EDC_DATA_PHY_BASE, ETHERCAT_OUTGOING_DATA_SIZE, EC_QUEUED);
	(*pd)[3] = EC_SyncMan(EC_PALM_EDC_CAN_BRIDGE_MASTER_IN_BASE, ETHERCAT_CAN_BRIDGE_DATA_SIZE, EC_QUEUED);



	(*pd)[0].ChannelEnable = true;
	(*pd)[0].ALEventEnable = true;
	(*pd)[0].WriteEvent = true;
	(*pd)[1].ChannelEnable = true;
	(*pd)[1].ALEventEnable = true;
	(*pd)[1].WriteEvent = true;

	(*pd)[2].ChannelEnable = true;
	(*pd)[3].ChannelEnable = true;

	sh->set_pd_config(pd);

	ROS_ERROR("status_size_ : %d ; command_size_ : %d\n", status_size_, command_size_);

	ROS_INFO("Finished to construct the SR06 driver");
}

int SR06::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{

  int retval = SR0X::initialize(hw, allow_unprogrammed);

//  com_ = EthercatDirectCom(EtherCAT_DataLinkLayer::instance());

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
  EthercatDevice::ethercatDiagnostics(d, 2);
}

void SR06::packCommand(unsigned char *buffer, bool halt, bool reset)
{
	static unsigned short int j = 0;
	static unsigned short int k = 0;
//	ROS_INFO("packCommand !");
	SR0X::packCommand(buffer, halt, reset);
	ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_INCOMING *command = (ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_INCOMING *)buffer;
	ETHERCAT_CAN_BRIDGE_DATA	*message = (ETHERCAT_CAN_BRIDGE_DATA *)(buffer + ETHERCAT_INCOMING_DATA_SIZE);
	signed short int motor[20] = {0};
	for (int i = 0 ; i < 20 ; ++i) {
		motor[i] = i << 8;
		motor[i] += (i + 1);
	}

//	motor[8] = 0x4143;
	if ( j < 2000)
	{
		ROS_ERROR("EDC_COMMAND_SENSOR_DATA !\n");
		command->EDC_command = EDC_COMMAND_SENSOR_DATA;
		++j;
	}
	else {
		k++;
		ROS_ERROR("EDC_COMMAND_CAN_TEST_MODE !\n");
		command->EDC_command = EDC_COMMAND_CAN_TEST_MODE;
	}
	memcpy(command->motor_torque_demand, motor, sizeof(command->motor_torque_demand));

	message->can_bus = 1;
	message->message_length = 2;
	message->message_id = 0x10; // CAN_NUM_DATA_REQUEST
//	for (int i = 0 ; i < message->message_length ; ++i)
//		message->message_data[i] = i;

	message->message_data[0] = k % 5; // asking for which_data 0, 1, 2, 3 and 4 
	message->message_data[1] = 0; // talking to motor_ID 0x08 , so which_motor must be 0
}

bool SR06::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING *tbuffer = (ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING *)(this_buffer + command_size_);
  ETHERCAT_CAN_BRIDGE_DATA *can_data = (ETHERCAT_CAN_BRIDGE_DATA *)(this_buffer + command_size_ + ETHERCAT_OUTGOING_DATA_SIZE);
  static unsigned int i = 0;
  static unsigned int errors = 0;
  static unsigned int num_rxed_packets = 0;
  unsigned int j;
/*uint32_t event_req;
uint32_t event_mask;
uint8_t spi_config;
EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());

readData(&com, 0x204, &event_mask, 4);
readData(&com, 0x220, &event_req, 4);
readData(&com, 0x150, &spi_config, 1);

ROS_ERROR("event_req == %08X", event_req);
ROS_ERROR("event_mask == %08X", event_mask);
ROS_ERROR("spi_config == %02X", spi_config);
*/
  ++num_rxed_packets;
  if (tbuffer->EDC_command == EDC_COMMAND_INVALID)
  {
    ROS_ERROR("Reception error detected : %d errors out of %d rxed packets\n", ++errors, num_rxed_packets);
  }
/*
  for (j = 0 ; j < 20 ; ++j)
  {
  	ROS_ERROR("Motor[%d] : torque == %hd ; SG_L == %hu ; SG_R == %hu ; temp == %hd ; current == %hd ; flags == %hu\n", j, tbuffer->motor[j].torque, tbuffer->motor[j].SG_L, tbuffer->motor[j].SG_R, tbuffer->motor[j].temperature, tbuffer->motor[j].current, tbuffer->motor[j].flags);
  }
 */
  ROS_ERROR("Motor[8] : torque == %hd ; SG_L == %hu ; SG_R == %hu ; temp == %hd ; current == %hd ; flags == %hu\n", tbuffer->motor[8].torque, tbuffer->motor[8].SG_L, tbuffer->motor[8].SG_R, tbuffer->motor[8].temperature, tbuffer->motor[8].current, tbuffer->motor[8].flags);

  ROS_ERROR("CAN debug : can_bus : %d ; message_length : %d ; message_id : 0x%04X ; message_data : 0x%02X 0x%02X 0x%02X 0x%02X\n", can_data->can_bus, can_data->message_length, can_data->message_id, can_data->message_data[0], can_data->message_data[1], can_data->message_data[2], can_data->message_data[3]);

  if (i == max_iter_const) { // 10 == 100 Hz
    i = 0;
    return true;
  } else if (i * nb_publish_by_unpack_const < nb_sensors_const) {
    std_msgs::Int16 msg;
    unsigned char j;
    for (j = 0 ; j < nb_publish_by_unpack_const ; ++j) 
  {
      unsigned short int k = i * nb_publish_by_unpack_const + j;
      msg.data = *((signed short int *)tbuffer + k + 2);
      if (realtime_pub_[k]->trylock()){
        realtime_pub_[k]->msg_ =  msg;
        realtime_pub_[k]->unlockAndPublish();
      }
   }

  }
  ++i;

  return true;
}

