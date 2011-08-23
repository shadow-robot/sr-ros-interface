/**
 * @file   srbridge.h
 * @author Yann Sionneau <yann.sionneau@gmail.com>, Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Aug 23 11:35:21 2011
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
 * @brief This is a ROS driver for the etherCAT bridge.
 */

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

