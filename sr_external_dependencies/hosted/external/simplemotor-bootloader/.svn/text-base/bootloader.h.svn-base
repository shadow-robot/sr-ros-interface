//
// © 2010 Shadow Robot Company Limited.
//
// FileName:        bootloader.h
// Dependencies:
// Processor:       PIC18
// Compiler:        MPLAB® C32
//
//  +------------------------------------------------------------------------+
//  | This file is part of The Shadow Robot PIC32 firmware code base.        |
//  |                                                                        |
//  | It is free software: you can redistribute it and/or modify             |
//  | it under the terms of the GNU General Public License as published by   |
//  | the Free Software Foundation, either version 3 of the License, or      |
//  | (at your option) any later version.                                    |
//  |                                                                        |
//  | It is distributed in the hope that it will be useful,                  |
//  | but WITHOUT ANY WARRANTY; without even the implied warranty of         |
//  | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          |
//  | GNU General Public License for more details.                           |
//  |                                                                        |
//  | You should have received a copy of the GNU General Public License      |
//  | along with this code repository. The text of the license can be found  |
//  | in Pic32/License/gpl.txt. If not, see <http://www.gnu.org/licenses/>.  |
//  +------------------------------------------------------------------------+
//
//
//
//
//  Doxygen
//  -------
//
//! @file
//!
//! This file contains the definitions of the bootloading commands.
//!
//! @group
//


#ifndef BOOTLOADER_H_INCLUDED
#define BOOTLOADER_H_INCLUDED

//! These are the different commands the bootloader accepts.
//! 
typedef enum
{
    WRITE_FLASH_DATA_COMMAND      = 0x00,
    READ_FLASH_COMMAND            = 0x01,
    ERASE_FLASH_COMMAND           = 0x02,
    RESET_COMMAND                 = 0x03,
    READ_VERSION_COMMAND          = 0x04,
    WRITE_FLASH_ADDRESS_COMMAND   = 0x05,
    MAGIC_PACKET                  = 0x0A
}BOOTLOADER_COMMAND;


#endif
