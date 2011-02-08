#include <srh_ethercat_hardware/srbridge.h>

#include <iomanip>
#include <sstream>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

PLUGINLIB_REGISTER_CLASS(0, SRBridge, EthercatDevice);

void SRBridge::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  SR0X::construct(sh, start_address);
  assert(sh_->get_product_code() == PRODUCT_CODE);

  ROS_INFO("Shadow Bridge configure -  %d @ %d", sh_->get_product_code(), sh_->get_ring_position());
  ShadowRobotHand::newHand(sh_->get_ring_position());

  EtherCAT_FMMU_Config *fmmu;
  fmmu = new EtherCAT_FMMU_Config(0);
  sh_->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(0); 
  sh_->set_pd_config(pd);

}

int SRBridge::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  int result = SR0X::initialize(hw, allow_unprogrammed);
  if (result != 0)
    return result;

  assert(sh_->get_product_code() == PRODUCT_CODE);
  
  if (device_offset_ != 0) 
  {
    ROS_FATAL("Device offset of SRBRIDGE should be 0 not %d", device_offset_);
    return -1;
  }

  EthercatDirectCom com(EtherCAT_DataLinkLayer::instance());
  uint16_t data, new_data;
  int rv;

  rv = readData(&com, (EC_UINT)EC_Slave_RD[PDI_Conf_reg].ado, (void *)&data, (EC_UINT)2);
  ROS_INFO("bridge port type: %s\n", data&1?"MII":"EBUS");

  rv = readData(&com, 0x100, &data, 2);
  if (rv != 0) ROS_ERROR("can't read open status");
    
  new_data = data & ~0xc000;

  rv = writeData(&com, 0x100, &new_data, 2);
  if (rv != 0) ROS_ERROR("can't write DL values");
    
  rv = readData(&com, 0x100, &data, 2);
  if (rv != 0) ROS_ERROR("can't read open status");

  return 0;
}

void SRBridge::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  std::ostringstream str;
  str << "SRBridge : " << std::setw(2) << std::setfill('0') << sh_->get_ring_position();
  d.name = str.str();
  str.str("");
  str << sh_->get_product_code() << '-' << sh_->get_serial();
  d.hardware_id = str.str();

  d.message = "";
  d.level = 0;

  d.clear();
  d.addf("Position", "%02d", sh_->get_ring_position());
  d.addf("Product code", "%08x", sh_->get_product_code());
  d.addf("Serial", "%08x", sh_->get_serial());
  d.addf("Revision", "%08x", sh_->get_revision());

  this->ethercatDiagnostics(d, 2); 
}
