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
#include <fcntl.h>
#include <stdio.h>
#include <pthread.h>
#include <bfd.h>

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

#define WRITE_FLASH_COMMAND	0x00
#define READ_FLASH_COMMAND	0x01
#define ERASE_FLASH_COMMAND	0x02
#define RESET_COMMAND		0x03
#define READ_VERSION_COMMAND	0x04

PLUGINLIB_REGISTER_CLASS(6, SR06, EthercatDevice);

#define check_for_pthread_mutex_init_error(x)	switch(x) \
						{ \
							case EAGAIN: \
								ROS_ERROR("The system temporarily lacks the resources to create another mutex : %s:%d", __FILE__, __LINE__); \
								exit(1); \
								break; \
							case EINVAL: \
								ROS_ERROR("The value specified as attribute is invalid for mutex init : %s:%d", __FILE__, __LINE__); \
								exit(1); \
								break; \
							case ENOMEM: \
								ROS_ERROR("The process cannot allocate enough memory to create another mutex : %s:%d", __FILE__, __LINE__); \
								exit(1); \
								break; \
							case 0: /* SUCCESS */ \
								break; \
							default: \
								ROS_ERROR("unknown error value, is this POSIX system ? : %s:%d", __FILE__, __LINE__); \
								exit(1); \
						}

#define unlock(x)	switch ( pthread_mutex_unlock(x) ) \
			{ \
				case EINVAL: \
					ROS_ERROR("The value specified as a mutex is invalid : %s:%d", __FILE__, __LINE__); \
					exit(1); \
					break; \
				case EPERM: \
					ROS_ERROR("The current thread does not hold a lock on the mutex : %s:%d", __FILE__, __LINE__); \
					exit(1); \
					break; \
			}

#define check_for_trylock_error(x)	if (x == EINVAL) \
					{ \
						ROS_ERROR("mutex error %s:%d", __FILE__, __LINE__); \
						exit(1); \
					}

bool SR06::SimpleMotorFlasher(sr_edc_ethercat_drivers::SimpleMotorFlasher::Request &req, sr_edc_ethercat_drivers::SimpleMotorFlasher::Response &res)
{
	int err;
	unsigned char cmd_sent;
	bfd *fd;
	bfd_byte *binary_content = NULL;
	asection *s;
	const char *section_name;
	unsigned int section_size = 0;
	unsigned int section_addr = 0;

	ROS_ERROR("DEBUT DU SERVICE\n");
	bfd_init();

	fd = bfd_openr(req.firmware.c_str(), NULL);
	if (fd == NULL)
	{
		ROS_FATAL("error opening the file %s", req.firmware.c_str());
	}
/*	for (s = fd->sections ; s ; s = s->next)
	{*/
	if (!bfd_check_format (fd, bfd_object)) {
		if (bfd_get_error () != bfd_error_file_ambiguously_recognized) {
			ROS_FATAL("Incompatible format");
		}
	}


	ROS_ERROR("firmware %s's format is : %s.", req.firmware.c_str(), fd->xvec->name);

	s = fd->sections;

/*	if (s == NULL)
		ROS_FATAL("This binary contains no section.");
*/
	if (bfd_get_section_flags (fd, s) & (SEC_LOAD))
	{
		if (bfd_section_lma (fd, s) == bfd_section_vma (fd, s))
		{
			section_name = bfd_section_name (fd, s);
			section_size = (unsigned int) bfd_section_size (fd, s);
			section_addr = (unsigned int) bfd_section_lma (fd, s);
			binary_content = (bfd_byte *)malloc(section_size);

			if (binary_content == NULL)
				ROS_FATAL("Error allocating memory for binary_content");

			bfd_get_section_contents(fd, s, binary_content, 0, bfd_section_size (fd, s));
		} else {
			ROS_FATAL("something went wrong while parsing %s.", req.firmware.c_str());
		}
	} else {
		ROS_FATAL("something went wrong while parsing %s.", req.firmware.c_str());
	}
//	}
	

	ROS_ERROR("Sending the ERASE FLASH command\n");
	// First we send the erase command
	cmd_sent = 0;
	while (! cmd_sent )
	{
		if ( !(err = pthread_mutex_trylock(&producing)) )
		{
			flashing = true;
			can_message_.message_length = 1;
			can_message_.can_bus = 1;
			can_message_.message_id = 0x0600 | (req.motor_id << 5) | ERASE_FLASH_COMMAND;
			cmd_sent = 1;
			unlock(&producing);
		}
		else
		{
			check_for_trylock_error(err);
		}
	}
	can_message_sent = false;
	while ( !can_message_sent )
	{
//		ros::Duration d(0, 10E7);
//		d.sleep();
		usleep(1);
	}
	sleep(2);
/*
	ROS_ERROR("Sending the WRITE FLASH command with the address\n");
	// Then we send the write command with the address
	cmd_sent = 0;
	while (! cmd_sent )
	{
		if ( !(err = pthread_mutex_trylock(&producing)) )
		{
			can_message_.message_length = 3;
			can_message_.can_bus = 1;
			can_message_.message_id = 0x0600 | (req.motor_id << 5) | WRITE_FLASH_COMMAND;
			can_message_.message_data[2] = 0x00;
			can_message_.message_data[1] = 0x04; // User application start address is 0x4C0
			can_message_.message_data[0] = 0xc0;
			cmd_sent = 1;
			unlock(&producing);
		}
		else
		{
			check_for_trylock_error(err);
		}
	}

	can_message_sent = false;
	while ( !can_message_sent )
	{
//		ros::Duration d(0, 10E7);
//		d.sleep();
//		usleep(100);
		sleep(2);
	}
*/
	unsigned int pos = 0;
	ROS_ERROR("Sending the firmware data\n");
	// Then we send the data to be written into PIC18F Flash memory	
//	while ( (size = read(fd, &buffer, 8))  > 0)
	while ( pos < ((section_size % 32) == 0 ? section_size : (section_size + 32 - (section_size % 32))) )
	{
		if ((pos % 32) == 0)
		{
			cmd_sent = 0;
			while (! cmd_sent )
			{
				if ( !(err = pthread_mutex_trylock(&producing)) )
				{
					can_message_.message_length = 3;
					can_message_.can_bus = 1;
					can_message_.message_id = 0x0600 | (req.motor_id << 5) | WRITE_FLASH_COMMAND;
					can_message_.message_data[2] = 0x00;
					can_message_.message_data[1] = 0x04 + ((pos + 0xc0) >> 8); // User application start address is 0x4C0
					can_message_.message_data[0] = 0xc0 + pos;
					ROS_ERROR("Sending write address : 0x%02X%02X%02X", can_message_.message_data[2], can_message_.message_data[1], can_message_.message_data[0]);
					cmd_sent = 1;
					unlock(&producing);
				}
				else
				{
					check_for_trylock_error(err);
				}
			}
			can_message_sent = false;
			while ( !can_message_sent )
			{
				usleep(1);
			}
			usleep(100);
		}
		
		cmd_sent = 0;
		while (! cmd_sent )
		{
			if ( !(err = pthread_mutex_trylock(&producing)) )
			{
				ROS_ERROR("Sending data ... position == %d", pos);
				can_message_.message_length = 8;
				can_message_.can_bus = 1;
				can_message_.message_id = 0x0600 | (req.motor_id << 5) | WRITE_FLASH_COMMAND;
				bzero(can_message_.message_data, 8);
				for (unsigned char j = 0 ; j < 8 ; ++j)
					can_message_.message_data[j] = (pos > section_size) ? 0 : *(binary_content + pos + j);
				pos += 8;
				cmd_sent = 1;
				unlock(&producing);
			}
			else
			{
				check_for_trylock_error(err);
			}
		}
		can_message_sent = false;
		while ( !can_message_sent )
		{
			usleep(1);
		}
		usleep(100);
	}

//	close(fd); // We do not need the file anymore
	bfd_close(fd);
	ROS_ERROR("Sending the RESET command to PIC18F");
	// Then we send the RESET command to PIC18F
	cmd_sent = 0;
	while (! cmd_sent )
	{
		if ( !(err = pthread_mutex_trylock(&producing)) )
		{
			can_message_.message_length = 0;
			can_message_.can_bus = 1;
			can_message_.message_id = 0x0600 | (req.motor_id << 5) | RESET_COMMAND;
			cmd_sent = 1;
			unlock(&producing);
		}
		else
		{
			check_for_trylock_error(err);
		}
	}
		

	can_message_sent = false;
	while ( !can_message_sent )
	{
		usleep(1);
	}
	usleep(100);

	flashing = false;

	ROS_ERROR("Flashing done");

	res.value = res.SUCCESS;
	return true;
}

SR06::SR06() : SR0X()//, com_(EthercatDirectCom(EtherCAT_DataLinkLayer::instance()))
{
	char topic_name[4];
	unsigned char i;
	int res;
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

	flashing = false;
	can_message_sent = false;
	res = pthread_mutex_init(&producing, NULL);

	check_for_pthread_mutex_init_error(res);

	serviceServer = nodehandle_.advertiseService("SimpleMotorFlasher", &SR06::SimpleMotorFlasher, this);
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
	int res;
//	static unsigned short int k = 0;
//	static unsigned long int flash_address = 0x00007FFE;
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
	if ( j < 20)
	{
//		ROS_ERROR("EDC_COMMAND_SENSOR_DATA !\n");
		command->EDC_command = EDC_COMMAND_SENSOR_DATA;
		++j;
	}
	else {
//		k++;
//		ROS_ERROR("EDC_COMMAND_CAN_TEST_MODE !\n");
		command->EDC_command = EDC_COMMAND_CAN_TEST_MODE;
/*		flash_address += 8;
		if (flash_address >= 0x00007FFE)
		{
			flash_address = 0x3a0; // starting of User Application
		}*/
	}
	memcpy(command->motor_torque_demand, motor, sizeof(command->motor_torque_demand));

	if (flashing && !can_message_sent) {
		if ( !(res = pthread_mutex_trylock(&producing)) ) {
			ROS_ERROR("We send a CAN message for flashing !");
			memcpy(message, &can_message_, sizeof(can_message_));
			can_message_sent = true;
			ROS_ERROR("Sending : SID : 0x%04X ; bus : 0x%02X ; length : 0x%02X ; data : 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", message->message_id, message->can_bus, message->message_length, message->message_data[0], message->message_data[1], message->message_data[2], message->message_data[3], message->message_data[4], message->message_data[5], message->message_data[6], message->message_data[7]);
			unlock(&producing);
		}
		else
		{
			ROS_ERROR("Mutex is locked, we don't send any CAN message !");
			check_for_trylock_error(res);
		}
	}
	else
	{
		message->can_bus = 1;
		message->message_id = 0x00;
		message->message_length = 0;
	}

/*	message->can_bus = 1;
	message->message_id = 0x10; // CAN_NUM_DATA_REQUEST
//	for (int i = 0 ; i < message->message_length ; ++i)
//		message->message_data[i] = i;

	if ( (flash_address % 32) == 0 )
	{
		message->message_data[0] = 0x00; // WRITE_FLASH_COMMAND
		message->message_data[1] = (flash_address & 0xff0000) >> 16;
		message->message_data[2] = (flash_address & 0xff00) >> 8;
		message->message_data[3] = (flash_address & 0xff);
		message->message_length = 4;
		flash_address = 0x000003a0;
	} else {
		message->message_data[0] = 0x00;
		message->message_data[1] = 0xDE;
		message->message_data[2] = 0xAD;
		message->message_data[3] = 0xBA;
		message->message_data[4] = 0xBE;
		message->message_data[5] = 0x01;
		message->message_data[6] = 0x02;
		message->message_data[7] = 0x03;
		message->message_length = 8;
	}
//	message->message_length = 1;
//	message->message_id = 0xffff;
//	message->message_data[0] = 0x08;
*/
}

bool SR06::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING *tbuffer = (ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_OUTGOING *)(this_buffer + command_size_);
//  ETHERCAT_CAN_BRIDGE_DATA *can_data = (ETHERCAT_CAN_BRIDGE_DATA *)(this_buffer + command_size_ + ETHERCAT_OUTGOING_DATA_SIZE);
  static unsigned int i = 0;
  static unsigned int num_rxed_packets = 0;
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
  //  ROS_ERROR("Reception error detected : %d errors out of %d rxed packets\n", ++errors, num_rxed_packets);
  }
/*
  for (j = 0 ; j < 20 ; ++j)
  {
  	ROS_ERROR("Motor[%d] : torque == %hd ; SG_L == %hu ; SG_R == %hu ; temp == %hd ; current == %hd ; flags == %hu\n", j, tbuffer->motor[j].torque, tbuffer->motor[j].SG_L, tbuffer->motor[j].SG_R, tbuffer->motor[j].temperature, tbuffer->motor[j].current, tbuffer->motor[j].flags);
  }
 */
//  ROS_ERROR("Motor[8] : torque == %hd ; SG_L == %hu ; SG_R == %hu ; temp == %hd ; current == %hd ; flags == %hu\n", tbuffer->motor[8].torque, tbuffer->motor[8].SG_L, tbuffer->motor[8].SG_R, tbuffer->motor[8].temperature, tbuffer->motor[8].current, tbuffer->motor[8].flags);

//  ROS_ERROR("CAN debug : can_bus : %d ; message_length : %d ; message_id : 0x%04X ; message_data : 0x%02X 0x%02X 0x%02X 0x%02X\n", can_data->can_bus, can_data->message_length, can_data->message_id, can_data->message_data[0], can_data->message_data[1], can_data->message_data[2], can_data->message_data[3]);

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

