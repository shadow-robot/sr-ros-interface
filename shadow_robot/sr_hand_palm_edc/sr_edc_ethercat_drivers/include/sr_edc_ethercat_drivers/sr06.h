#ifndef SR06_H
#define SR06_H

#include <ethercat_hardware/ethercat_device.h>
#include <sr_edc_ethercat_drivers/sr0x.h>

class SR06 : public SR0X
{
public:
  void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  int initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);
  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *);
  
  SR06();
  ~SR06();
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
protected:
  int counter_;
};

#endif /* SR06_H */

