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
//  | It is distributed in the hope that it will be useful,              |
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
//! This Node definition for the 0220 Palm EDC node
//! 
//! 
//! 
//! 
//! @addtogroup NodeName
//

#ifndef THIS_NODE_H_INCLUDED
#define THIS_NODE_H_INCLUDED

#define DUAL_CAN_AVAILABLE

#include "GenericTypeDefs.h"

#include <plib.h>
#include <peripheral/can.h>
#include <peripheral/timer.h>
#include "hardware/can/shadow_can.h"
#include "misc/typedefs_shadow.h"
#include "tests/assert_shadow.h"

#define NO_SENSOR_NAMES
#include "0220_palm_edc_ethercat_protocol.h"
#include "simple_can/simple_can.h"

/**
 *  The INCLUDE_IMPLEMENTATION_DETAILS define exposes implementation details to this_node.h
 *
 *  All other code should not be exposed to implementation details.
 */
#define INCLUDE_IMPLEMENTATION_DETAILS
    #include "hardware/spi/spi_32.h"
    #include "hardware/uart/uart_32.h"
#undef INCLUDE_IMPLEMENTATION_DETAILS



#ifndef  CAN_ERROR_CODES
    #error CAN_ERROR_CODES not defined
#endif

#ifndef  CAN_EVENT_CODES
    #error CAN_EVENT_CODES not defined
#endif

#ifndef  SIMPLE_CAN_ERROR_CODES
    #error SIMPLE_CAN_ERROR_CODES not defined
#endif


//! Used for reporting error events, or a change in error state.
//! E.G. the change from CAN1_HAPPY to CAN1_SAD
//
//! Error and event codes relevant to CAN are defined in the shadow_can.h etc.
//! this enum simply collects together the error codes relevant to this node.
typedef enum
{
    //NO_ERROR_CODE,
    CAN_ERROR_CODES,
    SIMPLE_CAN_ERROR_CODES
}ERROR_CODE;


//! Used for reporting general events, or a change in state.
//! E.G. the change from Idle mode to Active mode
//
//! Error and event codes relevant to CAN are defined in the shadow_can.h etc.
//! this enum simply collects together the event codes relevant to this node.
typedef enum
{
    //NO_EVENT_CODE,
    CAN_EVENT_CODES,
}EVENT_CODE;



void initialise_this_node(void);                            // ALL setup for this node, apart from what's explicitly listed in main.c

void handle_configuration_message(CAN_message* message);
void Service_EtherCAT_Packet(void);


// application defines
#define THIS_NODE_PRODUCT_CODE      0x0006
#define THIS_NODE_SERIAL_NUMBER     0x0015

#define SYSTEM_FREQ_HZ            80000000
#define PERIPHERAL_BUS_CLOCK_HZ   40000000
#define CAN_BUS_SPEED_HZ           1000000

#define NUM_PORTS            7
#define NUM_PINS_PER_PORT   16
#define NUM_LEDS             7
#define NUM_SPI_PORTS        2

#define ET1200_SPI_CHANNEL      0
#define CAN_BASE_ADR_MOTORS_TX  0x0200
#define CAN_BASE_ADR_MOTORS_RX  0x0300
#define FIND_FREE_CAN_BUFFER_MAX_TRIES 8


#include "internal_reporting/internal_reporting.h"
#include "hardware/ports/port_pin.h"
#include "hardware/spi/spi_32.h"
#include "hardware/spi/spi_parallel_32.h"
#include "hardware/i2c/i2c_32.h"
#include "hardware/eeprom/eeprom_i2c.h"
#include "hardware/et1200/et1200_interface.h"
#include "hardware/et1200/et1200_eeprom_contents.h"
#include "hardware/et1200/et1200.h"
#include "itg3200/itg3200.h"
#include "leds/leds.h"


#include "../support/svnversion.h"

void Send_Sensor_Data_Structure(void);
void Fill_Sensor_Data_Structure(I2C_MODULE I2Cm);

void Read_Commands_From_ET1200(void);
void Send_All_CAN_Messages(void);
void Translate_SOMI_Bits(void);
void Read_All_Sensors(void);
void Collect_All_CAN_Messages(void);
inline void write_status_data_To_ET1200(void);

void delay_us(int32u microseconds) ;
void delay_ms(int32u milliseconds);

int8u get_which_motors(void);
int8u get_from_motor_data_type(void);

void bad_CAN_message_seen(void);

//                                        FF      MF      RF      LF       KN     HEEL   TH       NOTHING
extern int8u palm_EDC_0200_sensor_mapping[64];
extern int64u node_id;

extern ETHERCAT_CAN_BRIDGE_DATA                        can_bridge_data_from_ROS;
extern ETHERCAT_CAN_BRIDGE_DATA                        can_bridge_data_to_ROS;

#define PALM_PCB_00

#ifdef PALM_PCB_00
    #define ET1200_CHIP_SELECT_PIN      'C', 14
    #define ET1200_RESET_PIN            'F',  3
    #define ET1200_EEPROM_PIN           'G',  2
    #define SPI_CS_PIN                  'B',  9
    #define SPI_CLOCK_PIN               'B', 15
    #define SPI_MOSI_PIN                'G',  9
    #define ET1200_SOMI_PIN             'D',  2
    #define LED_CAN1_TX_PIN             'D',  4
    #define LED_CAN1_RX_PIN             'D',  5
    #define LED_CAN1_ERR_PIN            'D',  0
    #define LED_CAN2_TX_PIN             'D',  7
    #define LED_CAN2_ERR_PIN            'D',  6
    #define LED_AL_ERR_PIN              'D', 11
    #define LED_CFG_PIN                 'D',  8

    #define SPI_PORT                    SPI_CHANNEL1A, 10, ET1200_chip_select
    #define I2C_PORT                    I2C1, 400000

    #define ALL_LED_BITS                

    #define ALL_LED_BITS_PORTB          0b0000000000000000
    #define ALL_LED_BITS_PORTC          0b0000000000000000
    #define ALL_LED_BITS_PORTD          0b0000100111110001
    #define ALL_LED_BITS_PORTE          0b0000000000000000
    #define ALL_LED_BITS_PORTF          0b0000000000000000

    #define     SPIP_INPUT_BIT_0        (porte & 0x0001)
    #define     SPIP_INPUT_BIT_1        (porte & 0x0002)
    #define     SPIP_INPUT_BIT_2        (porte & 0x0004)
    #define     SPIP_INPUT_BIT_3        (porte & 0x0008)
    #define     SPIP_INPUT_BIT_4        (porte & 0x0010)
    #define     SPIP_INPUT_BIT_5        (porte & 0x0020)
    #define     SPIP_INPUT_BIT_6        (porte & 0x0040)
    #define     SPIP_INPUT_BIT_7        (porte & 0x0080)
    
    #define     PIN_BIT(x)              (1<<x)
    
    #define     SPIP_CHIP_SELECT_UP     LATBSET = PIN_BIT(9);
    #define     SPIP_CHIP_SELECT_DOWN   LATBCLR = PIN_BIT(9);
    
    #define     SPIP_CLOCK_UP           LATBSET = PIN_BIT(15);
    #define     SPIP_CLOCK_DOWN         LATBCLR = PIN_BIT(15);
    
    #define     SPIP_CHIP_MOSI_UP       LATGSET = PIN_BIT(9);
    #define     SPIP_CHIP_MOSI_DOWN     LATGCLR = PIN_BIT(9);

    #define     SPI_BUF     SPI1ABUF
    #define     SPI_STAT    SPI1ASTAT
    #define     SPI_CON     SPI1ACON 
    #define     SPI_STATbits    SPI1ASTATbits

#endif


#ifdef PALM_PCB_01
    #define ET1200_CHIP_SELECT_PIN      'C', 14 /**/
    #define ET1200_RESET_PIN            'F',  3 /**/
    #define ET1200_EEPROM_PIN           'G',  2 /**/
    #define SPI_CS_PIN                  'D',  7 /**/
    #define SPI_CLOCK_PIN               'D',  6 /**/
    #define SPI_MOSI_PIN                'D',  5 /**/
    #define ET1200_SOMI_PIN             'G',  7 /**/
    #define LED_CAN1_TX_PIN             'D',  0 /**/
    #define LED_CAN1_RX_PIN             'D',  4 /**/
    #define LED_CAN1_ERR_PIN            'D',  8 /**/
    #define LED_CAN2_TX_PIN             'F',  4 /**/
    #define LED_CAN2_RX_PIN             'B', 15 /**/
    #define LED_CAN2_ERR_PIN            'F',  5 /**/
    #define LED_AL_ERR_PIN              'C', 13 /**/
    
    #define SPI_PORT                    SPI_CHANNEL2A, 10, ET1200_chip_select
    #define I2C_PORT                    I2C1, 400000

    #define ALL_LED_BITS_PORTB          0b1000000000000000
    #define ALL_LED_BITS_PORTC          0b0010000000000000
    #define ALL_LED_BITS_PORTD          0b0000000100010001
    #define ALL_LED_BITS_PORTE          0b0000000000000000
    #define ALL_LED_BITS_PORTF          0b0000000000100000

    
    #define     SPIP_INPUT_BIT_0        (porte & 0x0001)
    #define     SPIP_INPUT_BIT_1        (porte & 0x0002)
    #define     SPIP_INPUT_BIT_2        (porte & 0x0004)
    #define     SPIP_INPUT_BIT_3        (porte & 0x0008)
    #define     SPIP_INPUT_BIT_4        (porte & 0x0010)
    #define     SPIP_INPUT_BIT_5        (porte & 0x0020)
    #define     SPIP_INPUT_BIT_6        (porte & 0x0040)
    #define     SPIP_INPUT_BIT_7        (porte & 0x0080)
    
    #define     PIN_BIT(x)              (1<<x)
    
    #define     SPIP_CHIP_SELECT_UP     LATDSET = PIN_BIT(7);
    #define     SPIP_CHIP_SELECT_DOWN   LATDCLR = PIN_BIT(7);
    
    #define     SPIP_CLOCK_UP           LATDSET = PIN_BIT(6);
    #define     SPIP_CLOCK_DOWN         LATDCLR = PIN_BIT(6);
    
    #define     SPIP_CHIP_MOSI_UP       LATDSET = PIN_BIT(5);
    #define     SPIP_CHIP_MOSI_DOWN     LATDCLR = PIN_BIT(5);
    
    #define     SPI_BUF         SPI2ABUF
    #define     SPI_STAT        SPI2ASTAT
    #define     SPI_CON         SPI2ACON 
    #define     SPI_STATbits    SPI2ASTATbits

#endif


#endif
