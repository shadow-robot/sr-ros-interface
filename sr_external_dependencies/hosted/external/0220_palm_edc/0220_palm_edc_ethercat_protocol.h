//
// © 2010 Shadow Robot Company Limited.
//
// FileName:        this_node.h
// Dependencies:
// Processor:       PIC32
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
//  Doxygen
//  -------
//
//! @file
//!
//! The term "Command" means data going from the ROS PC to the Node on the robot
//! Previously known as "Incoming"
//!
//! The term "Status"  means data going from Node on the robot the to the ROS PC
//! Previously known as "Outgoing"
//!
//!
//! @addtogroup
//

#ifndef PALM_EDC_0220_ETHERCAT_PROTOCOL_H_INCLUDED
#define PALM_EDC_0220_ETHERCAT_PROTOCOL_H_INCLUDED

#define NUM_MOTORS 20


//! The host can request different types of data from the palm.
typedef enum
{
    EDC_COMMAND_INVALID = 0,                                    //!< Reading an empty mailbox on the ET1200 results in a zero.
    EDC_COMMAND_SENSOR_DATA,                                    //!< Normal operating value. Palm transmits ADC readings.
    EDC_COMMAND_SENSOR_CHANNEL_NUMBERS,                         //!< Instead of sending ADC readings, send the channel number, so the host can confirm the firmware is correct.
    EDC_COMMAND_SENSOR_ADC_CHANNEL_CS,                          //!< Instead of sending ADC readings, send the chip select channel, so the host can confirm the firmware is correct.
    EDC_COMMAND_CAN_DIRECT_MODE                                 //!< Might be used in the future for running automated tests inside the firmware.

    //EDC_COMMAND_TEST_RESULTS,                                   //!< Might be used in the future for running automated tests inside the firmware.
}EDC_COMMAND;



//! The host can request different types of data from the motors.
//! These values are inserted into bits [3..0] of the message ID
//! in the Data Request (Start Of Frame) message.
//!
//!
//! \htmlonly
//!<table border=1>
//!  <tr> <td>                    </td> <td colspan=2>Word 0</td> <td colspan=2>Word 1</td> </tr>
//!  <tr> <td>FROM_MOTOR_DATA_TYPE</td> <td>Byte0</td> <td>Byte1</td> <td>Byte2</td> <td>Byte3</td> </tr>
//!
//!  <tr> <td>MOTOR_DATA_SGL</td>                     <td colspan=2>Torque</td>             <td colspan=2>SGL</td>                      </tr>
//!  <tr> <td>MOTOR_DATA_SGR</td>                     <td colspan=2>Torque</td>             <td colspan=2>SGR</td>                      </tr>
//!  <tr> <td>MOTOR_DATA_PWM</td>                     <td colspan=2>Torque</td>             <td colspan=2>PWM</td>                      </tr>
//!  <tr> <td>MOTOR_DATA_FLAGS</td>                   <td colspan=2>Torque</td>             <td colspan=2>Flags</td>                    </tr>
//!  <tr> <td>MOTOR_DATA_CURRENT</td>                 <td colspan=2>Torque</td>             <td colspan=2>Current</td>                  </tr>
//!  <tr> <td>MOTOR_DATA_VOLTAGE</td>                 <td colspan=2>Torque</td>             <td colspan=2>Voltage</td>                  </tr>
//!  <tr> <td>MOTOR_DATA_TEMPERATURE</td>             <td colspan=2>Torque</td>             <td colspan=2>Temperature</td>              </tr>
//!  <tr> <td>MOTOR_DATA_CAN_NUM_RECEIVED</td>        <td colspan=2>Torque</td>             <td colspan=2>Num Rx</td>                   </tr>
//!  <tr> <td>MOTOR_DATA_CAN_NUM_TRANSMITTED</td>     <td colspan=2>Torque</td>             <td colspan=2>Num Tx</td>                   </tr>
//!  <tr> <td>MOTOR_DATA_SVN_REVISION</td>            <td colspan=2>Server revision</td>    <td colspan=2>This revision (top bit = modified)</td> </tr>
//!  <tr> <td>MOTOR_DATA_READBACK_LAST_CONFIG</td>    <td colspan=2>TO_MOTOR_DATA_TYPE</td> <td colspan=2>Config value</td>             </tr>
//!  <tr> <td>MOTOR_DATA_CAN_ERROR_COUNTERS</td>      <td colspan=2>Torque</td>             <td          >Tx Err</td> <td>Rx Err</td>   </tr>
//!</table>
//! \endhtmlonly
//!
//! @author Hugo Elias
typedef enum
{
    MOTOR_DATA_INVALID                  = 0x0,                  //!< For safety, this is not a valid request
    MOTOR_DATA_SGL                      = 0x1,                  //!< ADC reading of left strain gauge
    MOTOR_DATA_SGR                      = 0x2,                  //!< ADC reading of right strain gauge
    MOTOR_DATA_PWM                      = 0x3,                  //!< Current motor PWM duty cycle.
    MOTOR_DATA_FLAGS                    = 0x4,                  //!< See error_flag_names[]
    MOTOR_DATA_CURRENT                  = 0x5,                  //!< Current in milliamps
    MOTOR_DATA_VOLTAGE                  = 0x6,                  //!< Voltage in millivolts
    MOTOR_DATA_TEMPERATURE              = 0x7,                  //!< Temperature in 8.8 fixed point format
    MOTOR_DATA_CAN_NUM_RECEIVED         = 0x8,                  //!< Number of CAN messages received by this motor
    MOTOR_DATA_CAN_NUM_TRANSMITTED      = 0x9,                  //!< Number of CAN messages transmitted by this motor
    MOTOR_DATA_SLOW_MISC                = 0xA,                  //!< See FROM_MOTOR_SLOW_DATA_TYPE
    MOTOR_DATA_READBACK_LAST_CONFIG     = 0xB,                  //!< Torque=TO_MOTOR_DATA_TYPE.  Misc=config value.
    MOTOR_DATA_CAN_ERROR_COUNTERS       = 0xC,                  //!< LSB = TX error counter. MSB = RX error counter.

    MOTOR_DATA_PTERM                    = 0xD,                  //!< Internal proportional term from the torque controller / 256
    MOTOR_DATA_ITERM                    = 0xE,                  //!< Internal integral term from the torque controller / 256
    MOTOR_DATA_DTERM                    = 0xF                   //!< Internal derivative term from the torque controller / 256
}FROM_MOTOR_DATA_TYPE;


                                                                // Non serious flags
#define MOTOR_FLAG_BITS_CURRENT_CHOKE               0x000F
#define MOTOR_FLAG_BITS_EEPROM_WRITING              0x0010      //!< 1=EEPROM write currently in progress, don't update configuration.
#define MOTOR_FLAG_BITS_LAST_CONFIG_CRC_FAILED      0x0020
#define MOTOR_FLAG_BITS_LAST_CONFIG_OUT_OF_RANGE    0x0040

                                                                // Serious flags cause the motor to be switched off
#define MOTOR_FLAG_BITS_MOTOR_ID_IS_INVALID         0x0200
#define MOTOR_FLAG_BITS_NO_DEMAND_SEEN              0x0400
#define MOTOR_FLAG_BITS_SGL_FAULT                   0x0800
#define MOTOR_FLAG_BITS_SGR_FAULT                   0x1000
#define MOTOR_FLAG_BITS_A3950_NFAULT                0x2000
#define MOTOR_FLAG_BITS_EEPROM_CONFIG_BAD_CRC       0x4000
#define MOTOR_FLAG_BITS_OVER_TEMP                   0x8000


typedef enum
{
    MOTOR_SLOW_DATA_INVALID              = 0x0000,
    MOTOR_SLOW_DATA_SVN_REVISION         = 0x0001,
    MOTOR_SLOW_DATA_SVN_SERVER_REVISION  = 0x0002,
    MOTOR_SLOW_DATA_SVN_MODIFIED         = 0x0003,
    MOTOR_SLOW_DATA_SERIAL_NUMBER_LOW    = 0x0004,
    MOTOR_SLOW_DATA_SERIAL_NUMBER_HIGH   = 0x0005,
    MOTOR_SLOW_DATA_GEAR_RATIO           = 0x0006,
    MOTOR_SLOW_DATA_ASSEMBLY_DATE_YYYY   = 0x0007,
    MOTOR_SLOW_DATA_ASSEMBLY_DATE_MMDD   = 0x0008,

    MOTOR_SLOW_DATA_CONTROLLER_F         = 0x0009,
    MOTOR_SLOW_DATA_CONTROLLER_P         = 0x000A,
    MOTOR_SLOW_DATA_CONTROLLER_I         = 0x000B,
    MOTOR_SLOW_DATA_CONTROLLER_IMAX      = 0x000C,
    MOTOR_SLOW_DATA_CONTROLLER_D         = 0x000D,
    MOTOR_SLOW_DATA_CONTROLLER_DEADSIGN  = 0x000E,
    MOTOR_SLOW_DATA_CONTROLLER_FREQUENCY = 0x000F,

    MOTOR_SLOW_DATA_LAST                 = 0x000F
}FROM_MOTOR_SLOW_DATA_TYPE;

#ifndef NO_STRINGS													                    // The PIC compiler doesn't deal well with strings.

    static const char* slow_data_types[17] = {  "Invalid",
                                                "SVN revision",
                                                "SVN revision on server at build time",
                                                "Modified from SVN revision",
                                                "Serial number low",
                                                "Serial number high",
                                                "Motor gear ratio",
                                                "Assembly date year",
                                                "Assembly date month, day",
                                                "Controller F",
                                                "Controller P",
                                                "Controller I",
                                                "Controller D",
                                                "Controller Imax",
                                                "Controller D",
                                                "Controller deadband & sign",
                                                "Controller loop frequency Hz"
                                               };

#endif

#define NO_TORQUE_CONTROL_ERROR_FLAGS  (  MOTOR_FLAG_BITS_SGL_FAULT              \
                                        | MOTOR_FLAG_BITS_SGR_FAULT )

#define SERIOUS_ERROR_FLAGS            (  MOTOR_FLAG_BITS_NO_DEMAND_SEEN         \
                                        | MOTOR_FLAG_BITS_A3950_NFAULT           \
                                        | MOTOR_FLAG_BITS_EEPROM_CONFIG_BAD_CRC  \
                                        | MOTOR_FLAG_BITS_OVER_TEMP    )


#define NO_DEMAND_TIMEOUT_MS    20                                                      //!< If a motor doesn't see any Torque or PWM demand values,
                                                                                        //!  how long, in milliseconds, before it switches off the motor.

#ifndef NO_STRINGS													                    // The PIC compiler doesn't deal well with strings.

    //! These are the names of the bits in the MOTOR_DATA_FLAGS.
    //! error_flag_names[n] is the name of bit 'n' in MOTOR_DATA_FLAGS.
    static const char* error_flag_names[16] = { "Current choke bit 6",
                                                "Current choke bit 7",
                                                "Current choke bit 8",
                                                "Current choke bit 9",
                                                "EEPROM write in progress",
                                                "Last CRC didn't match configs",
                                                "Last config received was out of range",
                                                "undefined flag",
                                                "Motor ID is invalid",
                                                "No demand seen for more than 20ms",
                                                "Last configuration sent failed its CRC. Please resend",
                                                "Fault with strain gauge 0: left",
                                                "Fault with strain gauge 1: right",
                                                "A3950 H-bridge nFault asserted",
                                                "EEPROM contains bad CRC. Motor off.",
                                                "Motor over temperature"
                                               };
#endif



//! The host can send different types of data from the motors.
//! These can be either control demands, or configurations.
//! These values are inserted into bits [3..0] of the message ID
//! in the Motor data message.
typedef enum
{
    MOTOR_DEMAND_INVALID                = 0x0,                  //!< A zero is what happens if an EtherCAT packet doesn't get through, so it's considered a special case.
    MOTOR_DEMAND_TORQUE                 = 0x1,                  //!< Demanding torque activates the Torque PID loop
    MOTOR_DEMAND_PWM                    = 0x2,                  //!< Demanding PWM bypasses the Torque PID loop, and gives the exact PWM you asked for
                                                                //!  except where

    MOTOR_SYSTEM_RESET                  = 0x3,

    MOTOR_CONFIG_FIRST_VALUE            = 0x7,                  //!< This is the first TO_MOTOR_DATA_TYPE which is actually a configuration and should be stored in EEPROM
    MOTOR_CONFIG_MAX_PWM                = 0x7,                  //!< Put an upper limit on the absolute value of the motor PWM. Range [0..0x03FF]
    MOTOR_CONFIG_SG_REFS                = 0x8,                  //!< Strain gauge amp references. LSB = ref for SGL, MSB = ref for SGR.
    MOTOR_CONFIG_F                      = 0x9,                  //!< Feed forward gain
    MOTOR_CONFIG_P                      = 0xA,                  //!< Proportional gain
    MOTOR_CONFIG_I                      = 0xB,                  //!< Integral gain
    MOTOR_CONFIG_D                      = 0xC,                  //!< Derivative gain
    MOTOR_CONFIG_IMAX                   = 0xD,                  //!< Maximum integral windup
    MOTOR_CONFIG_DEADBAND_SIGN          = 0xE,                  //!< MSB=Deadband. LSB=sign.
    MOTOR_CONFIG_LAST_VALUE             = 0xE,                  //!< This is the last config (apart from the CRC, which is special)
    MOTOR_CONFIG_CRC                    = 0xF                   //!< Sending this value, if it matches the CRC of the configs
                                                                //!  above, causes the configs to take effect.
}TO_MOTOR_DATA_TYPE;

#define MOTOR_SYSTEM_RESET_KEY               0x5200             //!< | Motor ID.


#define MOTOR_DEMAND_TORQUE_RANGE_MIN       -0x7FFF
#define MOTOR_DEMAND_TORQUE_RANGE_MAX        0x7FFF

#define MOTOR_DEMAND_PWM_RANGE_MIN          -0x03FF
#define MOTOR_DEMAND_PWM_RANGE_MAX           0x03FF

#define MOTOR_CONFIG_F_RANGE_MIN             0x0000
#define MOTOR_CONFIG_F_RANGE_MAX             0x7FFF

#define MOTOR_CONFIG_P_RANGE_MIN             0x0000
#define MOTOR_CONFIG_P_RANGE_MAX             0x7FFF

#define MOTOR_CONFIG_I_RANGE_MIN             0x0000
#define MOTOR_CONFIG_I_RANGE_MAX             0x7FFF

#define MOTOR_CONFIG_D_RANGE_MIN             0x0000
#define MOTOR_CONFIG_D_RANGE_MAX             0x7FFF

#define MOTOR_CONFIG_IMAX_RANGE_MIN          0x0000
#define MOTOR_CONFIG_IMAX_RANGE_MAX          0x3FFF

#define MOTOR_CONFIG_DEADBAND_RANGE_MIN      0x00
#define MOTOR_CONFIG_DEADBAND_RANGE_MAX      0xFF

#define MOTOR_CONFIG_SIGN_RANGE_MIN          0x00
#define MOTOR_CONFIG_SIGN_RANGE_MAX          0x01


#ifndef NO_STRINGS													                    // The PIC compiler doesn't deal well with strings.

    //! These are the human-readable names of the different types of data.
    //! you can send to the motors.
    static const char* to_motor_data_type_names[16] = { "INVALID",
                                                        "Demand: Torque",
                                                        "Demand: PWM",
                                                        "INVALID",
                                                        "INVALID",
                                                        "INVALID",
                                                        "INVALID",
                                                        "Config: Maximum PWM value",
                                                        "Config: Strain gauge amp references. LSB = ref for SGL, MSB = ref for SGR",
                                                        "Config: Feed forward gain",
                                                        "Config: Proportional gain",
                                                        "Config: Integral gain",
                                                        "Config: Derivative gain",
                                                        "Config: Maximum integral windup",
                                                        "Config: MSB=Deadband. LSB=Sign",
                                                        "Config: CRC",
                                                        };
#endif


//! Then the motor is in debug mode, the host can send different
//! types of spoof sensor data to the motors.
//! These values are inserted into bits [3..0] of the message ID
//! in the Motor data message.
//!
//! NOT CURRENTLY IMPLEMENTED IN THE MOTORS.
typedef enum
{
    MOTOR_DEBUG_INVALID                 = 0x0,                  //!< Should not be used
    MOTOR_DEBUG_TORQUE_DEMAND           = 0x1,                  //!< Same as MOTOR_DEMAND_TORQUE
    MOTOR_DEBUG_SGL                     = 0x2,                  //!< Spoof the left strain gauge sensor
    MOTOR_DEBUG_SGR                     = 0x3,                  //!< Spoof the right strain gauge sensor
    MOTOR_DEBUG_CURRENT                 = 0x4,                  //!< Spoof the current sensor
    MOTOR_DEBUG_TEMPERATURE             = 0x5                   //!< Spoof the temperature sensor
}TO_MOTOR_DEBUG_TYPE;



//! Each motor sends back two 16-bit words of status data on the CAN bus.
//! Generically, those two words look like this.
typedef struct
{
    int16s              torque;                                 //!< Measured Motor Torque
    int16u              misc;                                   //!< Some other value, determined by motor_data_type
}MOTOR_DATA_PACKET;


//! This represents the top two bits [10..9] of the CAN message ID.
//! These bits tell us the type of the message.
typedef enum
{
    DIRECTION_DATA_REQUEST              = 0x0,                  //!< Requesting that motors send back status data (AKA Start of Frame)
    DIRECTION_TO_MOTOR                  = 0x1,                  //!< Message contains command data being sent to the motors
    DIRECTION_FROM_MOTOR                = 0x2,                  //!< Message contains status data from a motor
    DIRECTION_BOOTLOADER                = 0x3                   //!< Message has something to do with boot-loading.
}MESSAGE_DIRECTION;


#define MESSAGE_ID_DIRECTION_BITS       0b11000000000           //!< Bit mask specifying which bits of the CAN message ID are used for the MESSAGE_DIRECTION
#define MESSAGE_ID_MOTOR_ID_BITS        0b00111100000           //!< Bit mask specifying which bits of the CAN message ID are used for the motor ID [0..9]
#define MESSAGE_ID_ACK_BIT              0b00000010000           //!< Bit mask specifying which bits of the CAN message ID are used for the ACK bit (only for bootloading)
#define MESSAGE_ID_TYPE_BITS            0b00000001111           //!< Bit mask specifying which bits of the CAN message ID are used for the TO_MOTOR_DATA_TYPE or FROM_MOTOR_DATA_TYPE

#define MESSAGE_ID_DIRECTION_SHIFT_POS  9                       //!< Bit number of lowest bit of MESSAGE_ID_DIRECTION_BITS
#define MESSAGE_ID_MOTOR_ID_SHIFT_POS   5                       //!< Bit number of lowest bit of MESSAGE_ID_MOTOR_ID_BITS
#define MESSAGE_ID_MOTOR_ID_BOTTOM_BIT  0b00000100000           //!< Used to mask for even/odd motors
#define MESSAGE_ID_MOTOR_ID_TOP_2_BITS  0b00110000000           //!< Used to group motors into fours.


typedef struct
{
    int16u              data[8];                                //!< As yet unspecified
}TACTILE_SENSOR_STATUS;


#define SENSORS_NUM_0220  ((int)36)                             //!< The number of sensors in the robot.
                                                                //!  This needs to be a #define because it's used to dimension an array.

#define JOINTS_NUM_0220   ((int)28)                             //!< The number of joints in the hand
                                                                //!  This needs to be a #define for symmetry with SENSORS_NUM

#ifndef NO_STRINGS                                              //   The PIC compiler doesn't deal well with strings.

    static const char* joint_names[JOINTS_NUM_0220] = {  "FFJ0", "FFJ1", "FFJ2", "FFJ3", "FFJ4",
                                                         "MFJ0", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
                                                         "RFJ0", "RFJ1", "RFJ2", "RFJ3", "RFJ4",
                                                         "LFJ0", "LFJ1", "LFJ2", "LFJ3", "LFJ4","LFJ5",
                                                         "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
                                                         "WRJ1", "WRJ2"
                                                      };


    //! This array defines the names of the joints. The names and order should match the enum SENSOR_NAMES_ENUM.

    static const char* sensor_names[SENSORS_NUM_0220] = {"FFJ1",  "FFJ2",  "FFJ3", "FFJ4",	                     //  [00..03] ADC readings from First finger
                                                         "MFJ1",  "MFJ2",  "MFJ3", "MFJ4",                       //  [04..07] ADC readings from Middle finger
                                                         "RFJ1",  "RFJ2",  "RFJ3", "RFJ4",                       //  [08..11] ADC readings from Ring finger
                                                         "LFJ1",  "LFJ2",  "LFJ3", "LFJ4", "LFJ5",               //  [12..16] ADC readings from Little finger
                                                         "THJ1",  "THJ2",  "THJ3", "THJ4", "THJ5A", "THJ5B",     //  [17..22] ADC readings from Thumb
                                                         "WRJ1A", "WRJ1B", "WRJ2",                               //  [23..25] ADC readings from Wrist
                                                         "ACCX",  "ACCY",  "ACCZ",                               //  [26..28] ADC readings from Accelerometer
                                                         "GYRX",  "GYRY",  "GYRZ",                               //  [29..31] ADC readings from Gyroscope
                                                         "AN0",   "AN1",   "AN2",  "AN3"                         //  [32..35] ADC readings from auxillary ADC port.
                                                       };
#endif


//! This enum defines which ADC reading goes into which sensors[].
typedef enum
{
	FFJ1=0, FFJ2,  FFJ3, FFJ4,                      // [ 0...3]
	MFJ1,   MFJ2,  MFJ3, MFJ4,                      // [ 4...7]
	RFJ1,   RFJ2,  RFJ3, RFJ4,                      // [ 8..11]
	LFJ1,   LFJ2,  LFJ3, LFJ4, LFJ5,                // [12..16]
    THJ1,   THJ2,  THJ3, THJ4, THJ5A, THJ5B,        // [17..22]
    WRJ1A,  WRJ1B, WRJ2,                            // [23..25]

	ACCX, ACCY, ACCZ,                               // [26..28]
	GYRX, GYRY, GYRZ,                               // [29..32]

	AN0, AN1, AN2, AN3,                             // [31..35]
    IGNORE                                          // [36]
}SENSOR_NAME_ENUM;




typedef enum
{
      PALM_SVN_VERSION              =  0,
    SERVER_SVN_VERSION              =  1,

    TACTILE_SENSOR_FF_INSTALLED     =  2,
    TACTILE_SENSOR_MF_INSTALLED     =  3,
    TACTILE_SENSOR_RF_INSTALLED     =  4,
    TACTILE_SENSOR_LF_INSTALLED     =  5,
    TACTILE_SENSOR_TH_INSTALLED     =  6

}HARD_CONFIGURATION_INFORMATION;




//      ---------------
//      TACTILE SENSORS
//      ---------------


typedef enum                                                            // Shadow Sensors
{
    TACTILE_SENSOR_SHADOW_TYPE_NONE             = 0,
    TACTILE_SENSOR_SHADOW_TYPE_PST              = 1,
    TACTILE_SENSOR_SHADOW_TYPE_6_AXIS           = 2
}TACTILE_SENSOR_SHADOW_TYPE;

typedef enum                                                            // Shadow Sensors
{
    TACTILE_SENSOR_PST_DATA_TYPE_NONE                 = 0,
    TACTILE_SENSOR_PST_DATA_TYPE_PRESSURE_TEMPERATURE = 1,
    TACTILE_SENSOR_PST_DATA_TYPE_SVN_VERSIONS         = 2
}TACTILE_SENSOR_PST_DATA_TYPE;


typedef struct
{
    int16u  pressure;
    int16u  temperature;
    int16u  padding[6];
}TACTILE_SENSOR_SHADOW_PST_DATA_CONTENTS;



typedef enum                                                            // Syntouch
{
    TACTILE_SENSOR_SYNTOUCH_TYPE_NONE           = 0,
    TACTILE_SENSOR_SYNTOUCH_TYPE_2_3            = 1
}TACTILE_SENSOR_SYNTOUCH_TYPE;


typedef enum                                                            // Syntouch
{
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_NONE          = 0x0000,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_1   = 0x0001,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_2   = 0x0002,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_3   = 0x0003,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_4   = 0x0004,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_5   = 0x0005,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_6   = 0x0006,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_7   = 0x0007,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_8   = 0x0008,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_9   = 0x0009,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_10  = 0x000A,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_11  = 0x000B,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_12  = 0x000C,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_13  = 0x000D,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_14  = 0x000E,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_15  = 0x000F,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_16  = 0x0010,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_17  = 0x0011,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_18  = 0x0012,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_ELECTRODE_19  = 0x0013,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_PDC           = 0x0014,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_TAC           = 0x0015,
    TACTILE_SENSOR_SYNTOUCH_DATA_TYPE_TDC           = 0x0016
}TACTILE_SENSOR_SYNTOUCH_DATA_TYPE;


typedef struct
{
    int16u  pac;
    int16u  hall;
    int16u  misc[6];
}TACTILE_SENSOR_SYNTOUCH_DATA_CONTENTS;



typedef enum                                                            // Manufacturers
{
    TACTILE_SENSOR_MANUFACTURER_NONE            = 0,
    TACTILE_SENSOR_MANUFACTURER_SHADOW          = 1,
    TACTILE_SENSOR_MANUFACTURER_SYNTOUCH        = 2,
    TACTILE_SENSOR_MANUFACTURER_BIELEFELD       = 3
}TACTILE_SENSOR_MANUFACTURER;


typedef enum
{
    TACTILE_SENSOR_GENERIC_INFO_INVALID         = 0,
    TACTILE_SENSOR_GENERIC_INFO_SVN_REVISION    = 0x8001,
    TACTILE_SENSOR_GENERIC_INFO_SVN_MODIFIED    = 0x8002,
    TACTILE_SENSOR_GENERIC_INFO_SVN_SERVER      = 0x8003,
    TACTILE_SENSOR_GENERIC_INFO_PCB_VERSION     = 0x8004,
    TACTILE_SENSOR_GENERIC_INFO_SERIAL_NUMBER_L = 0x8005,
    TACTILE_SENSOR_GENERIC_INFO_SERIAL_NUMBER_H = 0x8006
}TACTILE_SENSOR_INFO_REQUEST;


#ifndef NO_STRINGS

    static const char* tactile_sensor_shadow_type_strings[4]  = {   "None",
                                                                    "Pressure Sensor Tactile",
                                                                    "6 Axis"};

    static const char* tactile_sensor_syntouch_type_strings[4] = {  "None",
                                                                    "BioTac 2.3"};



    static const char* tactile_sensor_manufacturer_strings[4] = {   "None",
                                                                    "Shadow Robot Company Ltd.",
                                                                    "Syntouch",
                                                                    "Bielefeld University"};

    static const char* tactile_sensor_generic_info_strings[7] = {   "Invalid",
                                                                    "SVN revision",
                                                                    "Revision is modified",
                                                                    "SVN revision on server at build time",
                                                                    "PCB version",
                                                                    "Part serial number low",
                                                                    "Part serial number high"};

#endif



//#if (int)IGNORE > SENSORS_NUM
//    #error Not enough sensors[] in ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS
//#endif

//! These are the data sent from the Palm to the host.
typedef struct
{
    EDC_COMMAND             EDC_command;                        //!< This tells us the contents of the data below.
                                                                //!< This value should be identical to the EDC_command
                                                                //!< value which arrived from the host in the previous
                                                                //!< EtherCAT packet

	int16u					sensors[SENSORS_NUM_0220+1];

    FROM_MOTOR_DATA_TYPE    motor_data_type;                    //!< Which data does motor[] contain?
                                                                //!< This value should agree with the previous value
                                                                //!< in ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND
    int16s                  which_motors;                       //!< 0: Even motor numbers.  1: Odd motor numbers
                                                                //!< This value should agree with the previous value
                                                                //!< in ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND

    int32u                  which_motor_data_arrived;           //!< Bit N set when motor CAN message arrives. Ideally, bits 0..19 get set
    int32u                  which_motor_data_had_errors;        //!< Bit N set when motor sends bad CAN message Ideally, no bits get set.

    MOTOR_DATA_PACKET       motor_data_packet[10];              //!< Data for 10 motors only. (Even ones or Odd ones)

    int32u                  tactile_data_type;
    int16u                  tactile_data_valid;                 //!< Bit 0: FF. Bit 4: TH.
    TACTILE_SENSOR_STATUS   tactile[5];                         //

    int16u                  idle_time_us;                       //!< The idle time from when the palm has finished dealing with one EtherCAT
																//!< packet, and the next packet arriving. Ideally, this number should be more than 50.

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS;



//! These are the data sent by the host.
typedef struct
{
    EDC_COMMAND             EDC_command;                        //!< What type of data should the palm send back in the next packet?

    FROM_MOTOR_DATA_TYPE    from_motor_data_type;               //!< Which data does the host want from the motors?
    int16s                  which_motors;                       //!< Which motors does the host want to read?
                                                                //!< 0: Even motor numbers.  1: Odd motor numbers

    TO_MOTOR_DATA_TYPE      to_motor_data_type;
    int16s                  motor_data[NUM_MOTORS];             //!< Data to send to motors. Typically torque/PWM demands, or configs.

    int32u                  tactile_data_type;                  //!< Request for specific tactile data

} __attribute__((packed)) ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND;

/*
#define ETHERCAT_COMMAND_HEADER_SIZE  (  sizeof(EDC_COMMAND)           \       //!< What's the minimum amount of
                                       + sizeof(FROM_MOTOR_DATA_TYPE)   \
                                       + sizeof(int16s)  )
*/

#define ETHERCAT_COMMAND_HEADER_SIZE  (  sizeof(EDC_COMMAND) + sizeof(FROM_MOTOR_DATA_TYPE) + sizeof(int16s)  )

#define ETHERCAT_STATUS_DATA_SIZE       sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_STATUS)
#define ETHERCAT_COMMAND_DATA_SIZE      sizeof(ETHERCAT_DATA_STRUCTURE_0200_PALM_EDC_COMMAND)


//! This packet allows the palm to transmit and receive CAN messages
//! on either CAN bus. One CAN message per EtherCAT packet only.
//! The CAN messages can be used for bootloading new code onto the motors,
//! or to configure the motor boards.
typedef struct
{
    int8u   can_bus;
    int8u   message_length;
    int16u  message_id;
    int8u   message_data[8];
} __attribute__((packed)) ETHERCAT_CAN_BRIDGE_DATA;

#define ETHERCAT_CAN_BRIDGE_DATA_SIZE   sizeof(ETHERCAT_CAN_BRIDGE_DATA)


//! | ETHERCAT_COMMAND_DATA | ETHERCAT_CAN_BRIDGE_DATA_COMMAND | ETHERCAT_STATUS_DATA | ETHERCAT_CAN_BRIDGE_DATA_STATUS |
//! |                       |                                  |                      |
//! |                       |                                  |                      ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS
//! |                       |                                  |
//! |                       |                                  ETHERCAT_STATUS_DATA_ADDRESS
//! |                       |
//! |                       ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS
//! |
//! ETHERCAT_COMMAND_DATA_ADDRESS
//!
//!

#define ETHERCAT_COMMAND_DATA_ADDRESS               0x1000
#define ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS    (ETHERCAT_COMMAND_DATA_ADDRESS            + ETHERCAT_COMMAND_DATA_SIZE)

#define ETHERCAT_STATUS_DATA_ADDRESS                (ETHERCAT_CAN_BRIDGE_DATA_COMMAND_ADDRESS + ETHERCAT_CAN_BRIDGE_DATA_SIZE)
#define ETHERCAT_CAN_BRIDGE_DATA_STATUS_ADDRESS     (ETHERCAT_STATUS_DATA_ADDRESS             + ETHERCAT_STATUS_DATA_SIZE)

//#define NUM_CONFIGS_REQUIRED 5

#define INSERT_CRC_CALCULATION_HERE 	crc_i = (int8u) (crc_result&0xff);          \
                                    	crc_i ^= crc_byte;                          \
                                    	crc_result >>= 8;                           \
                                    	if(crc_i & 0x01)	crc_result ^= 0x3096;   \
                                    	if(crc_i & 0x02)	crc_result ^= 0x612c;   \
                                    	if(crc_i & 0x04)	crc_result ^= 0xc419;   \
                                    	if(crc_i & 0x08)	crc_result ^= 0x8832;   \
                                    	if(crc_i & 0x10)	crc_result ^= 0x1064;   \
                                    	if(crc_i & 0x20)	crc_result ^= 0x20c8;   \
                                    	if(crc_i & 0x40)	crc_result ^= 0x4190;   \
                                    	if(crc_i & 0x80)	crc_result ^= 0x8320;

#endif
