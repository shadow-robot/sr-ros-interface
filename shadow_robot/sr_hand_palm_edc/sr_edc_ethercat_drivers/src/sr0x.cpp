/**
 * @file   sr0x.cpp
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

#include <sr_edc_ethercat_drivers/sr0x.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>

#include <math.h>

PLUGINLIB_REGISTER_CLASS(5, SR0X, EthercatDevice);

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
  ROS_DEBUG("Device #%02d: SR0%d (%#08x) Firmware Revision %d.%02d, PCB Revision %c.%02d, Serial #: %d",
            sh_->get_ring_position(),
            sh_->get_product_code() % 100,
            sh_->get_product_code(), fw_major_, fw_minor_,
            'A' + board_major_, board_minor_,
            sh_->get_serial());

  device_offset_ = sh_->get_ring_position();// - hand_->getBridgeRingPosition();

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

