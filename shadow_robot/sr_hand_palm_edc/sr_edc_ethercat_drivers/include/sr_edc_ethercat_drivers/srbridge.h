#ifndef SRBRIDGE_H
#define SRBRIDGE_H

#include <sr_edc_ethercat_drivers/sr0x.h>

class SRBridge : public SR0X
{
public:
  int initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed);
  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

  enum
  {
    PRODUCT_CODE = 0
  };
};

#endif /* SRBRIDGE_H */

