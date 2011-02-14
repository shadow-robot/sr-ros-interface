#ifndef SR06_H
#define SR06_H

#include <ethercat_hardware/ethercat_device.h>
#include <sr_edc_ethercat_drivers/sr0x.h>

class SR06 : public SR0X
{
public:
  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  int initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

  SR06();
//  ~SR06();

};

#endif /* SR06_H */

