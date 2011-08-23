/**
 * @file   sr0x.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Aug 23 11:36:54 2011
 *
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief Generic driver for a Shadow Robot EtherCAT Slave.
 * SR06 inherits from this class.
 *
 *
 */
#ifndef SR0X_H
#define SR0X_H

#include <ethercat_hardware/ethercat_device.h>
#include <sr_edc_ethercat_drivers/motor_trace_buffer.h>



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

  int device_offset_;      //!< Offset of device position from first device of Shadow Hand

protected:
  int command_base_;
  int status_base_;

};

#endif /* SR0X_H */

