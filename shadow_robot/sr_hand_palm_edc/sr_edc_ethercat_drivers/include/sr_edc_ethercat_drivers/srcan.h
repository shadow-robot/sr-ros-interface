#ifndef SRCAN_H
#define SRCAN_H

#include <sr_edc_ethercat_drivers/sr0x.h>

/**************************************************************************** 
   EC_SHADOWCAN node, providing a CAN mapping for one PALM node and power control
   Product ID 2
   */

struct ec_shadowcan_sensor_data {
  uint16_t last_command_number;
  uint32_t rx_message_count;
  int16_t dat[36];
  uint8_t io_status; // bits set in this are EC_SHADOWCAN_IO_STATUS bits
} __attribute__((packed));


struct ec_shadowcan_command_data {
  uint16_t command_number; // enum ec_shadowcan_commands
  uint16_t command_magic; // Must be EC_SHADOWCAN_COMMAND_MAGIC
} __attribute__((packed));

enum ec_shadowcan_commands { 
  EC_SHADOWCAN_NODE_ON=1,
  EC_SHADOWCAN_NODE_OFF=2,
  EC_SHADOWCAN_MOTOR_ON=3,
  EC_SHADOWCAN_MOTOR_OFF=4,
  EC_SHADOWCAN_ECAT_ON=5,
  EC_SHADOWCAN_ECAT_OFF=6,
};

class SRCAN : public SR0X
{
public:
  SRCAN();
  int initialize(pr2_hardware_interface::HardwareInterface *, bool allow_unprogrammed=true);
  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);
  enum
  {
    PRODUCT_CODE = EC_PRODUCT_ID_SHADOWCAN
  };

protected:
  static const unsigned EC_SHADOWCAN_COMMAND_PHY_BASE = 0x1000;
  static const unsigned EC_SHADOWCAN_SENSOR_PHY_BASE  = 0x1100;
  
  enum 
  {
    EC_SHADOWCAN_IO_STATUS_FAN_ON   = 1,
    EC_SHADOWCAN_IO_STATUS_MOTOR_ON = 2,
    EC_SHADOWCAN_IO_STATUS_NODE_ON  = 4,
    EC_SHADOWCAN_IO_STATUS_ECAT_ON  = 8,
  };

  static const uint16_t EC_SHADOWCAN_COMMAND_MAGIC=0xFFFF;

  void read_slave_data(char *text, struct ec_shadowcan_sensor_data *buff);
  void write_slave_command(char *text, struct ec_shadowcan_command_data *buff);

  uint32_t last_rx_message_count_;
  unsigned cycles_since_rx_change_;
  bool rx_stopped_updating_;
};

#endif /* SRCAN_H */

