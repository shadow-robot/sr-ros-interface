#include <srh_ethercat_hardware/sr0x.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>

#include <math.h>

PLUGINLIB_REGISTER_CLASS(5, SR0X, EthercatDevice);

SensorAdjustment::SensorAdjustment() : 
  gain_(1.0), 
  offset_(0.0) 
{ 
  // empty
}

SensorAdjustment::SensorAdjustment(double gain, double offset) : 
  gain_(gain), 
  offset_(offset) 
{ 
  // empty
}

double SensorAdjustment::adjust(double value) const
{
  return (value * gain_) + offset_;
}

bool SensorAdjustment::getParams(std::string actuator_name, std::string variable, const SensorAdjustment &default_adj)
{
  bool success = true;
  std::string key;

  std::ostringstream os;
  os << "srh/" << actuator_name << "/" << variable << "/offset";  
  if (ros::param::search(os.str(), key) == true)
  {
    ros::param::get(key, offset_);
  } else {
    ROS_ERROR("No parameter found for %s -- %s", actuator_name.c_str(), os.str().c_str());
    success = false;
  }

  os.str("");
  os << "srh/" << actuator_name << "/" << variable << "/gain";
  if (ros::param::search(os.str(), key) == true)
  {
    ros::param::get(key, gain_);
  } else {
    ROS_ERROR("No parameter found for %s -- %s", actuator_name.c_str(), os.str().c_str());
    success = false;
  }

  if (!success) 
  {
    ROS_ERROR("Using default parameters for  %s -- %s", actuator_name.c_str(), variable.c_str());
    gain_   = default_adj.gain_;
    offset_ = default_adj.offset_;
  }
  return success;
}

SensorAdjustment DualMotorActuator::default_force_sensor_adj_(1.0, 0.0);
SensorAdjustment DualMotorActuator::default_position_sensor_adj_(1.0, 0.0);
SensorAdjustment DualMotorActuator::default_current_sensor_adj_(1.0, 0.0);
SensorAdjustment DualMotorActuator::default_voltage_sensor_adj_(1.0, 0.0);
SensorAdjustment DualMotorActuator::default_temperature_sensor_adj_(1.0, 0.0);

DualMotorActuator::DualMotorActuator() : 
  pr2_hardware_interface::Actuator(),
  slow_motor_current_limit_(0.0),
  slow_effort_limit_(1.0),
  quick_motor_current_limit_(0.0),
  quick_effort_limit_(1.0),
  halt_overtemp_(false),
  last_sensor_read_count_(0),
  status_stopped_updating_(false),
  motor_trace_buffer_(1000)
{
  // empty
}

bool DualMotorActuator::getParam(double &value, std::string actuator_name, std::string variable, double default_value)
{
  std::ostringstream os;
  std::string key;
  os << "srh/" << actuator_name << "/" << variable;
  if (ros::param::search(os.str(), key) == true)
  {
    ros::param::get(key, value);
    return true;
  } else {
    ROS_ERROR("No parameter found for %s -- %s", actuator_name.c_str(), os.str().c_str());
    value = default_value;
    return false;
  }
}


void DualMotorActuator::initialize(const SRH_Actuator_Mapping &mapping)
{
  name_ = mapping.name;
  slow_motor_current_limit_ = mapping.slow_motor_current_limit;
  quick_motor_current_limit_ = mapping.quick_motor_current_limit;
  force1_sensor_adj_.getParams(name_,   "force_1",  default_force_sensor_adj_);
  force2_sensor_adj_.getParams(name_,   "force_2",  default_force_sensor_adj_);
  force3_sensor_adj_.getParams(name_,   "force_3",  default_force_sensor_adj_);
  position_sensor_adj_.getParams(name_, "position", default_position_sensor_adj_);
  current_sensor_adj_.getParams(name_,  "current",  default_current_sensor_adj_);
  voltage_sensor_adj_.getParams(name_,  "voltage", default_voltage_sensor_adj_);
  temperature_sensor_adj_.getParams(name_, "temperature", default_temperature_sensor_adj_);
  getParam(min_effort_, name_, "min_effort", 0.0);
  if ((min_effort_ < 0.0) || (min_effort_ >= 0.99))
  {
    ROS_ERROR("Invalid min effort of %f, using 0", min_effort_);
    min_effort_ = 0.0;
  }


  force_sensor_.name_ = string(mapping.name) + "_force";
  force_sensor_.state_.state_.resize(3,0.0);

  srh_ethercat_hardware::ActuatorInfo ai;
  ai.name = string(mapping.name);
  ai.slow_motor_current_limit  = mapping.slow_motor_current_limit;  
  ai.quick_motor_current_limit = mapping.quick_motor_current_limit;
  ai.duty_limit = double(0x300);  // TODO : replace hardcoded values with constants
  ai.max_duty = double(0x3FF);    // 
  motor_trace_buffer_.initialize(ai);
}

double DualMotorActuator::degreesToRadians(double degrees)
{
  return (degrees * M_PI) / double(180.0);
}

double DualMotorActuator::radiansToDegrees(double radians)
{
  return (radians * double(180.0))/M_PI;
}

double DualMotorActuator::sensorDataToAngle(uint16_t data) const
{
  double d_position = double(int16_t(data)) / 256.0;
  return position_sensor_adj_.adjust(degreesToRadians(d_position));
}

//! re-scale effort values using min_effort_ so that 
//   0.0      ---> 0.0
//   +epsilon ---> +min_effort
//   -epsilon ---> -min_effort
//   +1.0     ---> +1.0
//   -1.0     ---> -1.0
double DualMotorActuator::rescaleEffort(double effort_in) const
{
    double effort = effort_in;
    static const double epsilon = 0.01;
    if (effort_in > epsilon)  
    {
      effort = effort_in * (1.0 - min_effort_) + min_effort_;
    } 
    else if (effort_in < -epsilon)
    {
      effort = effort_in * (1.0 - min_effort_) - min_effort_;
    }
    return effort;
}


vector<ShadowRobotHand*> ShadowRobotHand::hands_;

ShadowRobotHand::ShadowRobotHand(unsigned srbridge_ring_position) :
  srbridge_ring_position_(srbridge_ring_position)
{
  ROS_INFO("Creating new ShadowRobotHand for bridge %d", srbridge_ring_position_);      

  for (unsigned actuator_index=0; actuator_index<SRH_NUM_ACTUATORS; ++actuator_index)
  {
    DualMotorActuator    *actuator(&actuators_[actuator_index]);
    const SRH_Actuator_Mapping &mapping(SRH_ACTUATOR_MAP[actuator_index]);
    assert(actuator_index == mapping.actuator_index);
    actuator->initialize(mapping);
  }
}

void ShadowRobotHand::newHand(unsigned srbridge_ring_position)
{
  ShadowRobotHand *new_hand = new ShadowRobotHand(srbridge_ring_position);
  hands_.push_back(new_hand);
}

ShadowRobotHand* ShadowRobotHand::getHand(unsigned device_ring_position)
{
  BOOST_FOREACH(ShadowRobotHand *hand, hands_)
  {
    int offset = device_ring_position - hand->srbridge_ring_position_;
    if (offset < 12)
    {
      return hand;
    }
  }
  return NULL;
}

DualMotorActuator* ShadowRobotHand::getActuator(unsigned actuator_index)
{
  return &actuators_[actuator_index];
}

DualMotorActuator* ShadowRobotHand::getActuator(unsigned device_offset, unsigned slave_offset)
{
  for (unsigned actuator_index = 0; actuator_index < SRH_NUM_ACTUATORS; actuator_index++)
  {
    const SRH_Actuator_Mapping *mapping(&SRH_ACTUATOR_MAP[actuator_index]);
    if ((mapping->device_offset == device_offset) && (mapping->slave_offset == slave_offset)) 
    {
      assert(mapping->actuator_index == actuator_index);
      return &actuators_[actuator_index];
    }
  }
  return NULL;

}

SR0X::SR0X() : EthercatDevice()
{
  
}

SR0X::~SR0X()
{
  delete sh_->get_fmmu_config();
  delete sh_->get_pd_config();
}

void SR0X::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  EthercatDevice::construct(sh,start_address);
}

int SR0X::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  //int result = EthercatDevice::initialize(hw, allow_unprogrammed);
  //if (result != 0)
  //return result;

  ROS_DEBUG("Device #%02d: SR0%d (%#08x) Firmware Revision %d.%02d, PCB Revision %c.%02d, Serial #: %d", 
            sh_->get_ring_position(),
            sh_->get_product_code() % 100,
            sh_->get_product_code(), fw_major_, fw_minor_,
            'A' + board_major_, board_minor_,
            sh_->get_serial());

  hand_ = ShadowRobotHand::getHand(sh_->get_ring_position());
  if (hand_ == NULL) 
  {
    ROS_FATAL("Cannot locate ShadowHand for device #%02d", sh_->get_ring_position());
    return -1;
  }
  device_offset_ = sh_->get_ring_position() - hand_->getBridgeRingPosition();

  return 0;
}

int SR0X::readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length)
{
  return EthercatDevice::readData(com, address, data, length, FIXED_ADDR);
}


int SR0X::writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length)
{
  return EthercatDevice::writeData(com, sh_, address, data, length, FIXED_ADDR);
}

