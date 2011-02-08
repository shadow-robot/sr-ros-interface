#include <srh_ethercat_hardware/srcan.h>

#include <iomanip>
#include <sys/time.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>


PLUGINLIB_REGISTER_CLASS(2, SRCAN, EthercatDevice);

SRCAN::SRCAN() :
  last_rx_message_count_(0),
  cycles_since_rx_change_(0),
  rx_stopped_updating_(false)
{
  //empty
}

void SRCAN::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  SR0X::construct(sh, start_address);
  assert(sh_->get_product_code() == PRODUCT_CODE);

  ROS_INFO("Shadow CAN ::configure - product %d @ %d", sh->get_product_code(), sh->get_ring_position());

  ROS_DEBUG("%s: %d .. \n", __FUNCTION__, __LINE__); 

  int rv;
  struct ec_shadowcan_sensor_data buff;
  
  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());

  rv = readData(&com, EC_SHADOWCAN_SENSOR_PHY_BASE, &buff, sizeof(buff));
  
  ROS_INFO("ShadowCAN: Fan %s - Motor %s - ECAT %s - Nodes %s\n",
           (buff.io_status & EC_SHADOWCAN_IO_STATUS_FAN_ON)?" On":"Off",
           (buff.io_status & EC_SHADOWCAN_IO_STATUS_MOTOR_ON)?" On":"Off",
           (buff.io_status & EC_SHADOWCAN_IO_STATUS_ECAT_ON)?" On":"Off",
           (buff.io_status & EC_SHADOWCAN_IO_STATUS_NODE_ON)?" On":"Off"
           );

  command_size_ = sizeof(struct ec_shadowcan_command_data);
  status_size_ = sizeof(struct ec_shadowcan_sensor_data);

  command_base_ = start_address;
  EC_FMMU *commandFMMU = new EC_FMMU(command_base_, // Logical start address
                                     sizeof(struct ec_shadowcan_command_data), // Logical length
                                     0x00, // Logical StartBit
                                     0x07, // Logical EndBit
                                     EC_SHADOWCAN_COMMAND_PHY_BASE, // Physical Start address
                                     0x00, // Physical StartBit
                                     false, // Read Enable
                                     true, // Write Enable
                                     true); // Enable

  start_address += sizeof(struct ec_shadowcan_command_data); // Logical length

  status_base_ = start_address;
  EC_FMMU *statusFMMU = new EC_FMMU(status_base_, // Logical start address
                                    sizeof(struct ec_shadowcan_sensor_data), // Logical length
                                    0x00, // Logical StartBit
                                    0x07, // Logical EndBit
                                    EC_SHADOWCAN_SENSOR_PHY_BASE, // Physical Start address
                                    0x00, // Physical StartBit
                                    true, // Read Enable
                                    false, // Write Enable
                                    true); // Enable

  start_address += sizeof(struct ec_shadowcan_sensor_data); // Logical length

  EtherCAT_FMMU_Config *fmmu;
  fmmu = new EtherCAT_FMMU_Config(2);

  (*fmmu)[0] = *commandFMMU;
  (*fmmu)[1] = *statusFMMU;
  sh->set_fmmu_config(fmmu);

  delete(statusFMMU);
  delete(commandFMMU);
  statusFMMU = NULL;
  commandFMMU = NULL;

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(0); // 4 if we use the MBX

  sh->set_pd_config(pd);
}

int SRCAN::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  int result = SR0X::initialize(hw, allow_unprogrammed);
  if (result != 0) 
    return result;

  if (device_offset_ != 1) 
  {
    ROS_FATAL("Device offset of SRCAN should be 1 not %d", device_offset_);
    return -1;
  }

  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());
  
  uint16_t data;
  int rv;

  /* Test port 1 for being open and try and open it */
  rv = readData(&com, 0x110, &data, 2);
 
  if (data & (1 << 5)) {
    /* Link present - good - is it up? */
    if (data & (1 << 10)) {
      /* loop closed - bad */
      ROS_ERROR("port 1 loop closed - no  hardware? ");
    } 
    else {
      if (data & (1 << 11)) {
        /* Link active - fine*/
      }
      else {
        ROS_ERROR("port 1 no comms");
      }
    }
  } 
  else {
    ROS_ERROR("ShadowCAN node - no physical link to next node!");
  }

  struct ec_shadowcan_sensor_data buff;
  
  rv = readData(&com, EC_SHADOWCAN_SENSOR_PHY_BASE, &buff, sizeof(buff));
  
  ROS_INFO("ShadowCAN: Fan %s - Motor %s - ECAT %s - Nodes %s\n",
          (buff.io_status & EC_SHADOWCAN_IO_STATUS_FAN_ON)?" On":"Off",
          (buff.io_status & EC_SHADOWCAN_IO_STATUS_MOTOR_ON)?" On":"Off",
          (buff.io_status & EC_SHADOWCAN_IO_STATUS_ECAT_ON)?" On":"Off",
          (buff.io_status & EC_SHADOWCAN_IO_STATUS_NODE_ON)?" On":"Off"
           );
  return 0;
}


void SRCAN::read_slave_data(char *text, struct ec_shadowcan_sensor_data *buff) 
{
  int rv;
  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());

  rv = readData(&com, EC_SHADOWCAN_SENSOR_PHY_BASE, buff, sizeof(struct ec_shadowcan_sensor_data));
}

void SRCAN::write_slave_command(char *text, struct ec_shadowcan_command_data *buff) 
{
  int rv;
  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());
  
  rv = writeData(&com, EC_SHADOWCAN_COMMAND_PHY_BASE, buff, sizeof(struct ec_shadowcan_command_data));
}


void SRCAN::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  if (reset) 
  {
    rx_stopped_updating_ = false;
  }
}

bool SRCAN::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  struct ec_shadowcan_sensor_data *can_sdata = (struct ec_shadowcan_sensor_data *)(this_buffer + command_size_);
  struct ec_shadowcan_sensor_data *can_sdata_prev = (struct ec_shadowcan_sensor_data *)(prev_buffer + command_size_);

  double timestamp = 0; 
  struct timeval tv;
  if (gettimeofday(&tv, NULL))
  {
    ROS_WARN("SR0X: Failed to get system time, timestamp in state will be zero");
  }
  else 
  {
    timestamp = double(tv.tv_sec) + double(tv.tv_usec) / 1.0e+6;
  }

  // SRCAN device collects angle data for every actuator.
  for (unsigned actuator_index = 0; actuator_index<SRH_NUM_ACTUATORS; ++actuator_index)
  {
    DualMotorActuator *actuator(hand_->getActuator(actuator_index));
    pr2_hardware_interface::ActuatorState *state(&actuator->state_); 
    unsigned can_ctrl_offset = SRH_ACTUATOR_MAP[actuator_index].can_ctrl_offset;
    state->position_     = actuator->sensorDataToAngle(can_sdata->dat[can_ctrl_offset]);


    // TODO : velocity estimate from position estimate is very jumpy and could use some filtering.
    // Also, there is a disconnect between motor velocity and joint velocity because of tensioners.
    // For a position control of the finger joint it more useful to control the velocity of the motor 
    // than the velocity of the finger.
    double position_prev = actuator->sensorDataToAngle(can_sdata_prev->dat[can_ctrl_offset]); 
    double velocity = (state->position_ - position_prev) / (timestamp - actuator->prev_state_timestamp_);
    actuator->prev_state_timestamp_ = state->timestamp_;
    state->timestamp_ = timestamp; 
    state->velocity_ = velocity;

    actuator->motor_trace_sample_.velocity = velocity;
    actuator->motor_trace_sample_.position = state->position_;
  }

  // When CAN controller stops updating joints positions, position controllers go crazy (possibly causing damage).
  // Use RX message count to detected if input data stops updating and disable motors before controllers cause havok.
  if (can_sdata->rx_message_count != last_rx_message_count_)
  {
    cycles_since_rx_change_ = 0;
  }
  else 
  {
    if (++cycles_since_rx_change_ > 10)
    {
      rx_stopped_updating_ = true;
    }
  }
  last_rx_message_count_ = can_sdata->rx_message_count;

  return !rx_stopped_updating_;
}

void SRCAN::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  struct ec_shadowcan_sensor_data *can_sdata = (struct ec_shadowcan_sensor_data *)(buffer + command_size_);

  std::ostringstream str;
  str << "SRCAN : " << std::setw(2) << std::setfill('0') << sh_->get_ring_position();
  d.name = str.str();
  str.str("");
  str << sh_->get_product_code() << '-' << sh_->get_serial();
  d.hardware_id = str.str();

  d.message = "";
  d.level = 0;

  if (rx_stopped_updating_) 
  {
    d.mergeSummary(d.ERROR, "Input data stopped updating");
  }

  d.clear();
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product code", "%08x", sh_->get_product_code());
  d.addf("Serial", "%08x", sh_->get_serial());
  d.addf("Revision", "%08x", sh_->get_revision());

  d.add("Fan",   (can_sdata->io_status & EC_SHADOWCAN_IO_STATUS_FAN_ON)?" On":"Off");
  d.add("Motor", (can_sdata->io_status & EC_SHADOWCAN_IO_STATUS_MOTOR_ON)?" On":"Off");
  d.add("ECAT",  (can_sdata->io_status & EC_SHADOWCAN_IO_STATUS_ECAT_ON)?" On":"Off");
  d.add("Nodes", (can_sdata->io_status & EC_SHADOWCAN_IO_STATUS_NODE_ON)?" On":"Off");

  d.addf("RX message count", "%u", can_sdata->rx_message_count);
  d.addf("Cycle(s) since update", "%u", cycles_since_rx_change_);

  //static uint32_t rx_message_count_prev = 0;
  //d.addf("Est msg freq", "%d", can_sdata->rx_message_count-rx_message_count_prev);
  //rx_message_count_prev = can_sdata->rx_message_count; //HACK : 

  this->ethercatDiagnostics(d, 2); 
}
