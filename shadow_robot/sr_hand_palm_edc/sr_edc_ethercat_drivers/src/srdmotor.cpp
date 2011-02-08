#include <srh_ethercat_hardware/srdmotor.h>

#include <iomanip>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

//include <boost/foreach.hpp>

PLUGINLIB_REGISTER_CLASS(3, SRDMotor, EthercatDevice);

SRDMotor::SRDMotor() : SR0X::SR0X()
{
  for (int slave_offset=0; slave_offset<2; ++slave_offset)
  {
    actuators_[slave_offset] = NULL;
  }
}

int SRDMotor::set_ref_for_strainguages(void)
{
    struct ec_motor_actuator_data ac[2];
    struct ec_motor_sensor_data sd[2];
    
    int slave_offset = 0;

    memset (ac, 0, sizeof(ac));
    memset (sd, 0, sizeof(sd));

    for (slave_offset = 0; slave_offset < 2; slave_offset++)
    {
        uint8_t jump=64, i=12;
        
        ac[slave_offset].hbridge_flags = HBRIDGE_FLAG_SLEEP_WAKE | HBRIDGE_FLAG_BRAKE_BRAKE;

        ac[slave_offset].ref0 = 128;           // Reset the offsets before doing the search
        ac[slave_offset].ref1 = 128;           // Reset the offsets before doing the search
        
        while (i--)
        {
            ROS_DEBUG("Sending: ref0 = %d -- ref1 = %d ", ac[slave_offset].ref0, ac[slave_offset].ref1);

            ac[slave_offset].command_number++;
            write_slave_command((char *)"", &ac[slave_offset], slave_offset);
            
            read_slave_data((char *)"", &sd[slave_offset], slave_offset);

            ROS_DEBUG("Recieved: SG_ref_0 = %d -- SG_ref_1 = %d ", sd[slave_offset].SG_ref_0, sd[slave_offset].SG_ref_1);
            ROS_DEBUG("Recieved: SG_precal_0 = %d -- SG_precal_1 = %d ", sd[slave_offset].SG_precal_0, sd[slave_offset].SG_precal_1);

            
            if (sd[slave_offset].SG_precal_0 < 1024) {
                ac[slave_offset].ref0 -= jump;
            }
            else {
                ac[slave_offset].ref0 += jump;
            }
            
            if (sd[slave_offset].SG_precal_1 < 1024) {
                ac[slave_offset].ref1 -= jump;
            }
            else {
                ac[slave_offset].ref1 += jump;
            }
            
            if (jump > 1) {
                jump >>= 1;
            }
        }
        
        ac[slave_offset].command_number++;
        ac[slave_offset].ref0_write_to_EEPROM = 1;
        ac[slave_offset].ref1_write_to_EEPROM = 1;

        ROS_DEBUG("Sending Final: ref0 = %d -- ref1 = %d ", ac[slave_offset].ref0, ac[slave_offset].ref1);

        write_slave_command((char *)"", &ac[slave_offset], slave_offset);
    }

    return 0;
}


void SRDMotor::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  SR0X::construct(sh, start_address);
  
  assert(sh_->get_product_code() == PRODUCT_CODE);

  ROS_INFO("Shadow Dual Motor configure - product %d @ %d", sh->get_product_code(), sh->get_ring_position());
    
  command_size_ = sizeof(struct ec_dualmotor_actuator_data);
  status_size_ = sizeof(struct ec_dualmotor_sensor_data);
  
  command_base_ = start_address;
  EC_FMMU *commandFMMU = new EC_FMMU(start_address, // Logical start address
                                     sizeof(struct ec_dualmotor_actuator_data),// Logical length
                                     0x00, // Logical StartBit
                                     0x07, // Logical EndBit
                                     EC_DUALMOTOR_COMMAND_PHY_BASE, // Physical Start address
                                     0x00, // Physical StartBit
                                     false, // Read Enable
                                     true, // Write Enable
                                     true); // Enable
  
  start_address += sizeof(struct ec_dualmotor_actuator_data);

  status_base_ = start_address;
  EC_FMMU *statusFMMU = new EC_FMMU(start_address, // Logical start address
                                    sizeof(struct ec_dualmotor_sensor_data), // Logical length
                                    0x00, // Logical StartBit
                                    0x07, // Logical EndBit
                                    EC_DUALMOTOR_DATA_PHY_BASE, // Physical Start address
                                    0x00, // Physical StartBit
                                    true, // Read Enable
                                    false, // Write Enable
                                    true); // Enable
  
  start_address += sizeof(struct ec_dualmotor_sensor_data);

  EtherCAT_FMMU_Config *fmmu;
  fmmu = new EtherCAT_FMMU_Config(2);  
  (*fmmu)[0] = *commandFMMU;
  (*fmmu)[1] = *statusFMMU;
  sh->set_fmmu_config(fmmu);
  
  delete(statusFMMU);
  delete(commandFMMU);
  statusFMMU = NULL;
  commandFMMU = NULL;
  
  // TODO : this device should really be using SyncManagers
  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(0); // 4 if we use the MBX
  
  sh->set_pd_config(pd);
}


int SRDMotor::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  int retval = SR0X::initialize(hw, allow_unprogrammed);
  if (retval != 0)
  {
    return retval;
  }

  if (device_offset_ < 2 || device_offset_ > 11) 
  {
    ROS_FATAL("Device offset of SRDMotor should be in range 2 to 11 not %d", device_offset_);
    return -1; 
  }

  // Determine actuator information
  for (int slave_offset=0; slave_offset<2; ++slave_offset)
  {
    DualMotorActuator *actuator(hand_->getActuator(device_offset_, slave_offset));
    if( actuator == NULL )
    {
      ROS_FATAL("No actuator for SRH Device #%02d, Motor %d", device_offset_, slave_offset);
      return -1;
    }

    actuators_[slave_offset] = actuator;

    // Register actuator with pr2_hardware_interface::HardwareInterface
    if (hw) 
    {
      if (!hw->addActuator(actuator))
      {
        ROS_FATAL("An actuator of the name '%s' already exists.", actuator->name_.c_str());
        return -1;
      }

      // Register force sensors with pr2_hardware_interface
      if (!hw->addAnalogIn(&actuator->force_sensor_))
      {
        ROS_FATAL("An analog input of the name '%s' already exists.", actuator->force_sensor_.name_.c_str());
        return -1;
      }
    }
  }

  struct ec_motor_actuator_data ac;
      
  ac.command_number = 0;
  ac.hbridge_duty = 0;

  write_slave_command((char*) "init", &ac, 0);
  write_slave_command((char*) "init", &ac, 1);

  set_ref_for_strainguages();


  // List analog devices
  /*
  if (device_offset_ == 11)
  {
    BOOST_FOREACH(const pr2_hardware_interface::AnalogInMap::value_type &pair, hw->analog_ins_)
    {
      ROS_INFO("Analog : %s", pair.first.c_str());
    }
  }
  */

  return retval;
}


void SRDMotor::read_slave_data(char *text, struct ec_motor_sensor_data *buff, int motor) 
{
  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());
  int rv;
  
  if (motor >= 2) 
      ROS_ERROR("%s: motor too big %d", __FUNCTION__, motor);

  rv = readData(&com, EC_DUALMOTOR_DATA_PHY_BASE+(motor*sizeof(struct ec_motor_sensor_data)), buff, sizeof(struct ec_motor_sensor_data));

  if (rv != 0) 
      ROS_ERROR("dual_motor: read_slave_data failed \n");
}

void SRDMotor::write_slave_command(char *text, struct ec_motor_actuator_data *buff, int motor) 
{
  int rv;
  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());
  
  if (motor >= 2) 
      ROS_ERROR("%s: motor too big %d", __FUNCTION__, motor);
  
  rv = writeData(&com, EC_DUALMOTOR_COMMAND_PHY_BASE+(motor*sizeof(struct ec_motor_actuator_data)), buff, sizeof(struct ec_motor_actuator_data));
  
  if (rv != 0) 
      ROS_ERROR("dual_motor: write_slave_data failed \n");
}



/** \brief Converts effort value into PWM duty value
 * 
 * Assumes effort ranges from -1.0 to +1.0.
 */
int SRDMotor::effortToDuty(double effort)
{
  // Limit effort to value between -1.0 and 1.0, since it might be possible for really large 
  // floats to cause problems when converte to an integer
  if (effort > 1.0)
    effort = 1.0;
  else if (effort < -1.0)
    effort = -1.0;

  // A duty of 1.0 corresponds to the max theoretical duty value
  int duty = effort * double(HBRIDGE_DUTY_LIMIT);

  // Make sure rounding of float when converting to integer value
  // does not produce value that is too large
  if (duty > HBRIDGE_DUTY_LIMIT)
    duty = HBRIDGE_DUTY_LIMIT;
  else if (duty < -HBRIDGE_DUTY_LIMIT)
    duty = -HBRIDGE_DUTY_LIMIT;

  return duty;
}

/** \brief Converts duty value into effort
 * 
 * Assumes effort should range from -1.0 to +1.0.
 */
double SRDMotor::dutyToEffort(int duty)
{
  double effort = double(duty) / double(HBRIDGE_DUTY_LIMIT);
  return effort;
}

void SRDMotor::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  SR0X::packCommand(buffer, halt, reset);
              
  /* FIXME: Ignoring halt and reset for now OR check for current usage of halt */
  struct ec_dualmotor_actuator_data *dmotor_actuator_data = (struct ec_dualmotor_actuator_data *)buffer;

  if (reset)
  {
    level_ = 0;
    reason_ = "OK";
  }

  for (unsigned slave_offset=0; slave_offset<2; ++slave_offset)
  {
    DualMotorActuator *actuator(actuators_[slave_offset]);
    pr2_hardware_interface::ActuatorCommand *cmd(&actuator->command_);
    ec_motor_actuator_data *m(&dmotor_actuator_data->m[slave_offset]);

    //int slave_offset = dualmotor_actuator_[j].slave_offset; //SR_ACTUATOR_MAP[i].slave_offset;
    bool tmp = cmd->enable_;
    if (halt) {
      cmd->enable_ = false;
    }
      
    if (reset) 
    {
      actuator->halt_overtemp_ = false;
      actuator->status_stopped_updating_ = false;
    }

    actuator->last_executed_effort = cmd->effort_;

    double effort = cmd->effort_;
    effort = actuator->rescaleEffort(effort);
    effort = saturate(-actuator->slow_effort_limit_,  actuator->slow_effort_limit_,  effort);
    effort = saturate(-actuator->quick_effort_limit_, actuator->quick_effort_limit_, effort);
    
    int16_t hbridge_duty = effortToDuty(effort);

    // Put limit on duty value when current becomes too high.     
    if (hbridge_duty > HBRIDGE_DUTY_LIMIT) {
      hbridge_duty = HBRIDGE_DUTY_LIMIT;
    }
    else if (hbridge_duty < -HBRIDGE_DUTY_LIMIT) {
      hbridge_duty = -HBRIDGE_DUTY_LIMIT;
    }      

    actuator->motor_trace_sample_.hbridge_duty = hbridge_duty;
    actuator->motor_trace_sample_.commanded_effort = cmd->effort_;

    m->hbridge_duty = hbridge_duty; //cmd[slave_offset /*j*/]->effort_ * 768 * 3;
    m->command_number++;
    
    if (cmd->effort_ && !halt && !actuator->halt_overtemp_) {
      m->hbridge_flags = HBRIDGE_FLAG_SLEEP_WAKE | HBRIDGE_FLAG_BRAKE_NOBRAKE;
    }
    else 
    {
      m->hbridge_duty = 0;
      m->hbridge_flags = HBRIDGE_FLAG_SLEEP_WAKE | HBRIDGE_FLAG_BRAKE_BRAKE;
    }
  
    cmd->enable_ = tmp;
  }
}

double SRDMotor::saturate(double minval, double maxval, double input)
{
  return std::max(minval, std::min(maxval, input));
}

bool SRDMotor::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  struct ec_dualmotor_sensor_data *dmotor_sensor_data = (struct ec_dualmotor_sensor_data *)(this_buffer + command_size_);

  bool rv = true;

  for (int slave_offset = 0; slave_offset < 2; ++slave_offset)
  {
    DualMotorActuator *actuator(actuators_[slave_offset]);           // dualmotor_actuator_[slave_offset].
    pr2_hardware_interface::ActuatorState *state(&actuator->state_); // state[slave_offset]->
    ec_motor_sensor_data *s(&dmotor_sensor_data->s[slave_offset]);   // dmotor_sensor_data->s[slave_offset].

    // TODO : shouldn't the device report being disabled when it is halted (due to an error)
    state->is_enabled_ = 1;

    state->device_id_ = sh_->get_ring_position();

    // This current value is based on regression based on comparing raw 
    // current value with average measure from current probe. 
    // current(mA) = 1.85*raw - 14.32
    double unadjusted_motor_current = double(s->motor_current)*1.85 - 14.32;
    unadjusted_motor_current *= 1e-3; // Convert current value from mA to Amps
    // Note current measurement is absolute, sensor cannot measure negative current values
    unadjusted_motor_current = std::max(0.0, unadjusted_motor_current);
    state->last_measured_current_ = actuator->current_sensor_adj_.adjust(unadjusted_motor_current);

    // Limit duty/effort when motor current becomes too high
    // Use 2 different limits :
    //   a quick high-current limit
    //   a slow low-current limit
    // Limit unadjusted motor current, in case current scaling values are messed up
    // Back off a percentange of current overshoot every cycle motor is over-current
    {
      // The slow limit prevents motor from overheating when commanded past an end-stop or grasping an object.
      // The slow limit backs engages very slowly, but backs off quickly (25x faster).  
      // This allows finger to move in opposite direction after gripping an object.
      if (fabs(unadjusted_motor_current > actuator->slow_motor_current_limit_))
        actuator->slow_effort_limit_ += -0.0001;
      else 
        actuator->slow_effort_limit_ += 0.0025;
      actuator->slow_effort_limit_  = saturate(0.0, 1.0, actuator->slow_effort_limit_);
    }
    {
      // Having a short term limit allows motor to overcome friction when trying to move, 
      // but prevents motor or from ripping actuator apart.
      // The quick limit use a I-controller to accurately limit current closely at limit
      static const double I_gain = 0.02;
      actuator->quick_effort_limit_ += I_gain * (actuator->quick_motor_current_limit_ - fabs(unadjusted_motor_current));
      actuator->quick_effort_limit_ = saturate(0.0, 1.0, actuator->quick_effort_limit_);
    }
       
    state->last_executed_effort_ = actuator->last_executed_effort;
    state->last_measured_effort_ = dutyToEffort(s->last_hbridge);

    // Nominal motor SUPPLY voltage is 24V
    // Motor supply voltage is scaled by voltage divider and then fed into uC ADC.
    // R12 = 5.6k  R13=1k  : 10bit ADC with 5V Vref
    static const double R12=5.6e3;
    static const double R13=1.0e3;
    static const double VOLTAGE_SCALING_FACTOR = 5.0 / 1023.0 * ((R12+R13)/R13);
    double unadjusted_voltage = double(s->motor_voltage) * VOLTAGE_SCALING_FACTOR;
    state->motor_voltage_  = actuator->voltage_sensor_adj_.adjust(unadjusted_voltage);

    std::vector<double> &force_sensor(actuator->force_sensor_.state_.state_);
    force_sensor[0] = actuator->force1_sensor_adj_.adjust(s->force_sensor_1);
    force_sensor[1] = actuator->force2_sensor_adj_.adjust(s->force_sensor_2);
    force_sensor[2] = actuator->force3_sensor_adj_.adjust((int16_t)s->force_sensor_3);

    double unadjusted_temperature = double(s->temperature_val) / 256.0;
    actuator->temperature_ = actuator->temperature_sensor_adj_.adjust(unadjusted_temperature);
    
    // Add measured data as motor trace sample
    {
      srh_ethercat_hardware::MotorTraceSample &s(actuator->motor_trace_sample_);
      s.slow_effort_limit = actuator->slow_effort_limit_;
      s.quick_effort_limit = actuator->quick_effort_limit_;
      s.motor_current = state->last_measured_current_;
      s.motor_supply_voltage = state->motor_voltage_;
      s.temperature = actuator->temperature_;
      s.force_sensor_1 = force_sensor[0];
      s.force_sensor_2 = force_sensor[1];
      s.force_sensor_3 = force_sensor[2];
      s.motor_velocity = 0.0; //motor_velocity;
      actuator->motor_trace_buffer_.sample(s);
      actuator->motor_trace_buffer_.checkPublish();
    }


    // Use both adjusted and un-adjusted temperatures in case adjustment gain/offset value totally wrong (ie gain=0).
    if ((actuator->temperature_ >= MOTOR_HALT_TEMP) || 
        (unadjusted_temperature >= MOTOR_HALT_TEMP)  ) 
    {
      if (!actuator->halt_overtemp_) 
      {
        ROS_WARN("Motor %d on slave # %d has halted due to high temperature ", slave_offset, sh_->get_ring_position());
        actuator->motor_trace_buffer_.flagPublish("Over temperature", 2 /*ERROR*/, 100);
      }
      actuator->halt_overtemp_ = true;
      state->halted_ = true;
    }

    // It is possible for one or both motor boards to connected to ethercat device to stop working.
    // Use the sensor_read_count to detect this condition.
    if (s->sensor_read_count != actuator->last_sensor_read_count_)
    {
      actuator->cycles_since_status_change_ = 0;
    }
    else 
    {
      if (++actuator->cycles_since_status_change_ > 100)
      {
        actuator->status_stopped_updating_ = true;
      }
    }
    actuator->last_sensor_read_count_ = s->sensor_read_count;

    // Return error if this motor is overtemperature, or we stopped getting status updates
    if (actuator->halt_overtemp_ || actuator->status_stopped_updating_)
    {
      rv = false;
    }
  }

  return rv;
}


bool SRDMotor::publishTrace(const string &reason, unsigned level, unsigned delay)
{
  // Publish traces for both motors attached to EtherCAT device.
  for (int slave_offset = 0; slave_offset < 2; ++slave_offset)
  {
    DualMotorActuator *actuator(actuators_[slave_offset]);
    //ROS_INFO("SRDMotor : Publishing trace for %s", actuator->name_.c_str());
    actuator->motor_trace_buffer_.flagPublish(reason, level, delay);
  }
  return true;
}


void SRDMotor::multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer)
{
  const struct ec_dualmotor_actuator_data *raw_actuator = (const ec_dualmotor_actuator_data*) buffer;
  const struct ec_dualmotor_sensor_data   *raw_sensor   = (const ec_dualmotor_sensor_data*)(buffer+command_size_);
  diagnostic_updater::DiagnosticStatusWrapper &d(diagnostic_status_);

  // EtherCAT device diagnostics //
  std::ostringstream str;
  str << "SRDMotor : " << std::setw(2) << std::setfill('0') << sh_->get_ring_position();
  d.name = str.str();
  str.str("");
  str << sh_->get_product_code() << '-' << sh_->get_serial();
  d.hardware_id = str.str();
  d.summary(d.OK, "OK");
  d.clear();
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product code", "%08x", sh_->get_product_code());
  d.addf("Serial", "%08x", sh_->get_serial());
  d.addf("Revision", "%08x", sh_->get_revision());
  this->ethercatDiagnostics(d, 2); 
  vec.push_back(d);

  // Motor diagnostics (2 motors)
  for (unsigned slave_offset=0; slave_offset<2; ++slave_offset)
  {
    const DualMotorActuator *actuator(actuators_[slave_offset]);
    const ec_motor_actuator_data *m(&raw_actuator->m[slave_offset]);
    const ec_motor_sensor_data *s(&raw_sensor->s[slave_offset]);
    const pr2_hardware_interface::ActuatorState *state(&actuator->state_);

    str.str("");
    str << "SRDMotor : " << std::setw(2) << std::setfill('0') << sh_->get_ring_position() 
        << " (" << actuators_[slave_offset]->name_ << ")";
    d.name = str.str();
    d.summary(d.OK, "OK");

    if (actuator->halt_overtemp_) 
    {
      d.mergeSummary(d.ERROR, "Motor over temperature");
    }

    if (actuator->status_stopped_updating_)
    {
      d.mergeSummary(d.ERROR, "Status inputs stopped updating");
    }

    d.clear();
    d.addf("Slave offset", "%d", slave_offset);
    d.addf("Commanded Duty", "%d", m->hbridge_duty);
    d.addf("Measured Duty", "%d", s->last_hbridge);
    d.addf("Command number", "%d", m->command_number);
    d.addf("H-bridge flags", "%s,%s (%x)", 
           (m->hbridge_flags & HBRIDGE_FLAG_BRAKE_MASK) ? "BRAKE" : "NOBRAKE",
           (m->hbridge_flags & HBRIDGE_FLAG_SLEEP_MASK) ? "WAKE" : "SLEEP",
           m->hbridge_flags);

    d.addf("Command effort", "%f", actuator->command_.effort_);
    d.addf("Scaled effort",  "%f", actuator->rescaleEffort(actuator->command_.effort_));
    d.addf("Minimum effort", "%f", actuator->min_effort_);
    d.addf("Slow effort limit", "%f", actuator->slow_effort_limit_);
    d.addf("Quick effort limit", "%f", actuator->quick_effort_limit_);

    d.addf("Joint position (degrees)", "%f", DualMotorActuator::radiansToDegrees(state->position_));
    d.addf("Joint velocity (degrees/s)", "%f", DualMotorActuator::radiansToDegrees(state->velocity_));

    d.addf("Sensor read count", "%d", s->sensor_read_count);
    d.addf("Cycle since update", "%d", actuator->cycles_since_status_change_);

    d.addf("Motor current (mA)"  , "%f (%d)", 1000.0 * state->last_measured_current_, s->motor_current);
    d.addf("Motor slow current limit (mA)", "%f",  1000.0 * actuator->slow_motor_current_limit_);
    d.addf("Motor quick current limit (mA)", "%f", 1000.0 * actuator->quick_motor_current_limit_);
    d.addf("Motor supply voltage", "%f (%d)", state->motor_voltage_, s->motor_voltage);
    d.addf("Motor temperature"   , "%f (%d)", actuator->temperature_, s->temperature_val);

    const std::vector<double> &force_sensor(actuator->force_sensor_.state_.state_);
    d.addf("Force sensor 1", "%f", force_sensor[0]);
    d.addf("Force sensor 2", "%f", force_sensor[1]);
    d.addf("Force sensor 3", "%f", force_sensor[2]);

    d.addf("SG_ref_0",    "%d", s->SG_ref_0);
    d.addf("SG_zero_0",   "%d", s->SG_zero_0);
    d.addf("SG_precal_0", "%d", s->SG_precal_0);

    d.addf("SG_ref_1",    "%d", s->SG_ref_1);
    d.addf("SG_zero_1",   "%d", s->SG_zero_1);
    d.addf("SG_precal_1", "%d", s->SG_precal_1);

    d.addf("Magic", "%x", s->magic);

    d.addf("Time delta (ms)", "%f", (state->timestamp_ - actuator->prev_state_timestamp_)*1000.0); 

    vec.push_back(d);
  }
}
