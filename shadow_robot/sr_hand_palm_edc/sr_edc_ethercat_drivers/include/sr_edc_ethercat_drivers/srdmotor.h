#ifndef SRDMOTOR_H
#define SRDMOTOR_H

#include <sr_edc_ethercat_drivers/sr0x.h>

/**************************************************************************** 
   EC_DUALMOTOR node, running two motors
   Product ID 3
   */

/* Communication notes:
   Write command to actuator_data[X].
   Wait a short time.
   Read sensor_data[X].
   Check last_command_executed == command_number.
*/

struct ec_motor_actuator_data
{
    int16_t   hbridge_duty;
    uint16_t  hbridge_flags;

    uint8_t   command_number;
    uint8_t   ref0;
    uint8_t   ref1;
    uint8_t   magic;

    uint8_t   ref0_write_to_EEPROM;
    uint8_t   ref1_write_to_EEPROM;
}__attribute__((packed));


struct ec_motor_sensor_data
{
    uint16_t force_sensor_1, force_sensor_2, force_sensor_3;
    uint16_t motor_current, motor_voltage, temperature_val;
    uint16_t station_alias;
    uint16_t sensor_read_count; //!< Incremented on each update of the values.
    int16_t  last_hbridge;
    uint16_t dummy;
    uint8_t  last_command_executed;
    uint8_t  magic;

    uint8_t  SG_ref_0, SG_ref_1;
    int16_t  SG_zero_0, SG_zero_1;
    uint16_t SG_precal_0, SG_precal_1;
}__attribute__((packed));

struct ec_dualmotor_actuator_data {
  struct ec_motor_actuator_data m[2];
}   
__attribute__((packed));

struct ec_dualmotor_sensor_data {
  struct ec_motor_sensor_data s[2];
}
__attribute__((packed));


class SRDMotor : public SR0X
{
public:

  SRDMotor();
  int set_ref_for_strainguages();
  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  int initialize(pr2_hardware_interface::HardwareInterface *, bool allow_unprogrammed=true);
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
  void multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer);
  bool publishTrace(const string &reason, unsigned level, unsigned delay);

  enum
  {
    PRODUCT_CODE = 3
  };

protected:   
  static const unsigned EC_DUALMOTOR_COMMAND_PHY_BASE=0x1040;
  static const unsigned EC_DUALMOTOR_DATA_PHY_BASE=0x1000;

  enum 
  {
    HBRIDGE_FLAG_BRAKE_MASK=1,
    HBRIDGE_FLAG_BRAKE_BRAKE=1,
    HBRIDGE_FLAG_BRAKE_NOBRAKE=0,
    HBRIDGE_FLAG_SLEEP_MASK=2,
    HBRIDGE_FLAG_SLEEP_WAKE=2,
    HBRIDGE_FLAG_SLEEP_SLEEP=0,
  };

  static const int HBRIDGE_DUTY_LIMIT = 0x300;  //<! Software enforced Duty Limit For HBridge. 
                                                //<! Duty is limited to +/- DUTY_LIMIT
  static const int HBRIDGE_MAX_DUTY   = 0x3FF;  //<! Maximum duty value value of 0x3FF represent %100 duty
  static const unsigned HBRIDGE_SLEEP_TIME = 0x80;   //<! How long before hbridge_duty==0 causes
                                       //<! me to put the hbridge into sleep mode. 0xFF max

  static int effortToDuty(double effort);
  static double dutyToEffort(int duty);
  static double saturate(double minval, double maxval, double input);

  static const double MOTOR_HALT_TEMP = 48.0; /* Equal to 48 degree C */

  void read_slave_data(char *text, struct ec_motor_sensor_data *buff, int motor);
  void write_slave_command(char *text, struct ec_motor_actuator_data *buff, int motor);

  DualMotorActuator *actuators_[2];
};

#endif /* SRDMOTOR_H */

