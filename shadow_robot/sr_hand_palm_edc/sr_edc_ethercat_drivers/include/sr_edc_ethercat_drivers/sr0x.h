#ifndef SR0X_H
#define SR0X_H

#include <ethercat_hardware/ethercat_device.h>
#include <sr_edc_ethercat_drivers/motor_trace_buffer.h>

/* Defined for naming actuators on ShadowHand */
enum {SRH_NUM_ACTUATORS = 20};

struct SRH_Actuator_Mapping {
  unsigned actuator_index; //!< Index into array of acturator for shadow hand 
  unsigned device_offset; //!< ET1200 slave number (offset from ECBridge)
  unsigned slave_offset; //!< number for finding Motor 0 or 1 attached to slave
  int can_ctrl_offset;  //!< Offset in can controller position data array
  int can_ctrl_offset1;//!< Offset in can controller position data array in case of compound joint
  double slow_motor_current_limit; //!< When motor current is higher, slowly reduce effort limit
  double quick_motor_current_limit; //!< When motor current is higher, quickly reduce effort limit
  const char *name; //!< name for actuator
};

/* FIXME: Have to check slave_offset when testing that which is 0 and which is 1 */
/* FIXME: Have to check can_ctrl_offset when testing that is the current value is valid or not */
const struct SRH_Actuator_Mapping SRH_ACTUATOR_MAP[SRH_NUM_ACTUATORS] = {
  { 0,  2, 0,  6, -1, 0.160, 0.340, "srh_mfj3"}, //mfj3: motor 1
  { 1,  2, 1, 25, -1, 0.160, 0.340, "srh_mfj0"}, //mfj0: motor 2
  { 2,  3, 0,  7, -1, 0.160, 0.340, "srh_mfj4"}, //mfj4: motor 3
  { 3,  3, 1, 26, -1, 0.160, 0.340, "srh_rfj0"}, //rfj0: motor 4

  { 4,  4, 0, 11, -1, 0.160, 0.340, "srh_rfj4"}, //rfj4: motor 5
  { 5,  4, 1, 10, -1, 0.160, 0.340, "srh_rfj3"}, //rfj3: motor 6
  { 6,  5, 0, 17, -1, 0.160, 0.340, "srh_thj1"}, //thj1: motor 7
  { 7,  5, 1, 14, -1, 0.160, 0.340, "srh_lfj3"}, //lfj3: motor 8
 
  { 8,  6, 0, 27, -1, 0.160, 0.340, "srh_lfj0"}, //lfj0: motor 9
  { 9,  6, 1, 15, -1, 0.160, 0.340, "srh_lfj4"}, //lfj4: motor 10
  {10,  7, 0, 18, -1, 0.160, 0.340, "srh_thj2"}, //thj2: motor 11
  {11,  7, 1, 16, -1, 0.160, 0.340, "srh_lfj5"}, //lfj5: motor 12
 
  {12,  8, 0, 24, -1, 0.160, 0.340, "srh_ffj0"}, //ffj0: motor 13
  {13,  8, 1,  2, -1, 0.160, 0.340, "srh_ffj3"}, //ffj3: motor 14
  {14,  9, 0,  3, -1, 0.160, 0.340, "srh_ffj4"}, //ffj4: motor 15
  {15,  9, 1, 19, -1, 0.160, 0.340, "srh_thj3"}, //thj3: motor 16

  {16, 10, 0, 23, -1, 0.300, 0.600, "srh_wrj2"}, //wrj2: motor 18
  {17, 10, 1, 22, -1, 0.300, 0.600, "srh_wrj1"}, //wrj1: motor 17

  {18, 11, 0, 21, -1, 0.300, 0.600, "srh_thj5"}, //thj5: motor 20
  {19, 11, 1, 20, -1, 0.300, 0.600, "srh_thj4"}  //thj4: motor 19
};


class SensorAdjustment
{
public:
  SensorAdjustment(); 
  SensorAdjustment(double gain, double offset); 

  /*!
   * \brief Applies gain/offset adjustment to sensor value
   */
  double adjust(double value) const;
  
  /*!
   * \brief Loads offset and gain values from parameter server for given actuator variable
   * If either offset or gain parameter cannot be loaded, will use default_adj values instead.
   * Paramater names are in form : /srh/actuator_name/variable/(offset|gain)
   * Example : /srh/srh_ffj0/position/offset
   * \param actuator_name name of actuator ie sr_dmotor_ffj0
   * \param variable_name name variable 
   * \return true if both gain and offset parameters were found
   */
  bool getParams(std::string actuator_name, std::string variable_name, const SensorAdjustment &default_adj);

protected:
  double gain_;
  double offset_;
};


class DualMotorActuator : public pr2_hardware_interface::Actuator
{
public:
  DualMotorActuator();

  /*!
   * \brief Initializes DualMotorActuator.  
   *        Tries pulling actuator gain/offset settings from parameter server using actuator name
   */
  void initialize(const SRH_Actuator_Mapping &mapping);
  static double degreesToRadians(double degrees);
  static double radiansToDegrees(double radians);
  int16_t angles_to_duty(double effort_);

  /*!
   * \brief Converts sensor data to actutator angle.  
   *        Uses position_sensor_adj to adjust value before returning it.
   * \param data  Sensor data from SRCAN sensor board
   * \return Actuator position (in radians) 
   */
  double sensorDataToAngle(uint16_t data) const;
  
  double   last_executed_effort; //!< The actual effort requested after safety limits were enforced

  //! Magnitude limit for motor current. Scale back motor effort slowly when current is a
  double slow_motor_current_limit_; 
  //! Slow moving magnitude limit for actuator effort. Dynamically scaled when motor current is too high.  
  double slow_effort_limit_; 

  //! Magnitude limit for motor current. Scale back motor effort quickly when current is above this value
  double quick_motor_current_limit_;
  //! Fast moving magnitude limit for actuator effort. Dynamically scaled when motor current is above this value
  double quick_effort_limit_;

  double min_effort_; //!< Minimum effort to use for actuator, when effort > +/-0.01, use +/- min effort
  double rescaleEffort(double effort_in) const;

  double prev_state_timestamp_; //!< timestamp of previous status, so that we can calculate velocity

  double temperature_; //!< Motor temperature in celcius (adjusted)

  bool halt_overtemp_; //!< Set to true when motor has halted due to over temperature condition.  Clear on reset-motors

  uint16_t last_sensor_read_count_; //!< Use sensor read count to make sure srdmotor is providing new status data
  unsigned cycles_since_status_change_;
  bool status_stopped_updating_;

  //! Force sensor inputs   
  // Index 0 contains force_1 : first strain gauge
  // Index 1 contains force_2 : second strain gauge
  // Index 2 contains force_3 : difference between strain gaugesx
  // Note all force values have their own gain/offset adjustment so : 
  //   force_1 != force_2 - force_3 
  // after adjustments are made
  pr2_hardware_interface::AnalogIn force_sensor_;

  SensorAdjustment force1_sensor_adj_;
  SensorAdjustment force2_sensor_adj_;
  SensorAdjustment force3_sensor_adj_;
  SensorAdjustment temperature_sensor_adj_;
  SensorAdjustment position_sensor_adj_;
  SensorAdjustment current_sensor_adj_;
  SensorAdjustment voltage_sensor_adj_;

  // Default offset/gain adjustment values
  static SensorAdjustment default_force_sensor_adj_;
  static SensorAdjustment default_position_sensor_adj_;
  static SensorAdjustment default_current_sensor_adj_;
  static SensorAdjustment default_voltage_sensor_adj_;
  static SensorAdjustment default_temperature_sensor_adj_;

  sr_edc_ethercat_drivers::MotorTraceBuffer motor_trace_buffer_;
  sr_edc_ethercat_drivers::MotorTraceSample motor_trace_sample_;

protected:
  static bool getParam(double &value, std::string actuator_name, std::string variable, double default_value);
};


// Stores all actuator information for a single Shadow Robot Hand.
class ShadowRobotHand
{
public:  

  /*!
   * \brief Creates a new ShadowRobotHand object and adds it to list of ShadowRobotHands.
   * \param srbridge_ring_position  EtherCAT ring position of Shadow Bridge device.
   */
  static void newHand(unsigned srbridge_ring_position);

  /*!
   * \brief Returns a pointer to ShadowRobotHand object for device at given ring position
   * \param device_ring_position  EtherCAT ring position of Shadow EtherCAT device.
   * \return Pointer to shadow hand device is part of.  NULL if appropriate hand is not found.
   */
  static ShadowRobotHand *getHand(unsigned device_ring_position);

  DualMotorActuator* getActuator(unsigned actuator_index);
  DualMotorActuator* getActuator(unsigned device_offset, unsigned slave_offset);

  unsigned getBridgeRingPosition() const {return srbridge_ring_position_;}

protected:
  ShadowRobotHand(unsigned srbridge_ring_position);
  DualMotorActuator actuators_[SRH_NUM_ACTUATORS]; //! All actuators for a single robot hand
  unsigned srbridge_ring_position_; //! Ring postion of Shadow Hand Bridge.  This allows differentiation between multiple Shadow Robot Hands. 

  static vector<ShadowRobotHand*> hands_;
};


class SR0X : public EthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

  SR0X();
  virtual ~SR0X();
protected:
  uint8_t fw_major_;
  uint8_t fw_minor_;
  uint8_t board_major_;
  uint8_t board_minor_;
  
  enum
  {
    MODE_OFF = 0x00,
    MODE_ENABLE = (1 << 0),
    MODE_CURRENT = (1 << 1),
    MODE_SAFETY_RESET = (1 << 4),
    MODE_SAFETY_LOCKOUT = (1 << 5),
    MODE_UNDERVOLTAGE = (1 << 6),
    MODE_RESET = (1 << 7)
  };

  enum 
  {
    EC_PRODUCT_ID_BRIDGE    = 0,
    EC_PRODUCT_ID_SHADOWCAN = 2,
    EC_PRODUCT_ID_DUALMOTOR = 3,
  };

  string reason_;
  int level_;

  int writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length);
  int readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length);

  ShadowRobotHand *hand_; //!< Shadow robot hand that device is part off
  int device_offset_;      //!< Offset of device position from first device of Shadow Hand

protected:
  int command_base_;
  int status_base_;

};

#endif /* SR0X_H */

