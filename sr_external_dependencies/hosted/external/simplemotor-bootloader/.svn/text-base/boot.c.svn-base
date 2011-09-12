#include "basics.h"
#include <p18f2580.h>


#define CAN_SJW         0x01	// range 1..4       Synchronisation Jump Width
#define CAN_BRP         0x02	// range 1..64      Baud Rate Prescaler
#define CAN_PHSEG1      0x01	// range 1..8       Phase Segment 1
#define CAN_PHSEG2      0x02	// range 1..8       Phase Segment 2
#define CAN_PROPSEG     0x06    // range 1..8       Propagation Segment
#define CAN_SAM         0x00	// range 0 or 1     Sample type (0==once, 1==three times)
#define CAN_SEG2PHTS    0x00	// range 0 or 1     

#define CAN_CONFIG_MODE     0b10000000
#define CAN_LISTEN_MODE	    0b01100000
#define CAN_LOOPBACK_MODE   0b01000000
#define CAN_DISABLE_MODE    0b00100000
#define CAN_NORMAL_MODE     0b00000000

#define RD          0
#define RXFUL       7
#define TXREQ       3
#define EEPGD       7
#define WREN        2
#define WR          1
#define CFGS        6
#define GIE         7

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

#define WAITING_FOR_ADDRESS             0x00
#define BUFFERING_DATA                  0x01

// The motor_id, hardcoded, this will have to be put in EEPROM in the future.
int8u motor_id = 0xFF;



#pragma code VIntH=0x0008

//! This function remaps the High Interrupt Vector
//! 
//! @author Yann Sionneau
void VIntH(void)
{
    _asm
        goto 0x0270 
    _endasm
}

#pragma code


#pragma code VIntL=0x000e

//! This function remaps the Low Interrupt Vector
//! 
//! @author Yann Sionneau
void VIntL(void)
{
    _asm
        goto 0x0280
    _endasm
}



#pragma code

//#pragma code BOOT

typedef enum _ECAN_RX_MSG_FLAGS
{
    ECAN_RX_OVERFLOW     = 0b00001000,
    ECAN_RX_INVALID_MSG  = 0b00010000,
    ECAN_RX_XTD_FRAME    = 0b00100000,
    ECAN_RX_RTR_FRAME    = 0b01000000,
    ECAN_RX_DBL_BUFFERED = 0b10000000

} ECAN_RX_MSG_FLAGS;

struct CANmsg 
{
	int16u messageID;
	union msg_data
	{
          int8u byte[8];
          int16u word[4];
          int32u dword[2];
	      int64u qword;
	} d;
	int8u length;
	ECAN_RX_MSG_FLAGS flags;
} CanMsgR; // declares the reception CAN message

struct CANmsg CanMsgT; // declares the transmit CAN message

int8u position; // this is the position of the current FLASH_WRITTING operation ( 0 <= position <= 32)


static int8u read_eeprom(int8u address)
{
    EECON1 = 0;
    EEADR = address;
    //EEADRH = 0xff; FIXME: why is this in the microchip code ?
    EECON1 |= (1 << RD);
    return EEDATA;
}

//! This checks if we should boot (start the user application), or stay in bootloader mode. 
//! We check the last byte of EEPROM if it is 0xFF it means no firmware has been
//! written in user application memory and we need to stay in bootloader mode,
//! waiting for commands to flash a user application through the CAN bootloading protocol.
//! If it is NOT 0xFF, it means there is a firmware and we should boot directly on it.
//!
//! @author Yann Sionneau
static int8u we_should_boot(void)
{
    return (read_eeprom(0xFF) != 0xFF);
}


//! Used for configuration of the CAN peripheral
//! The sync jump width affects how much adjustment
//! is allowed when synchronising to the bus edges.
typedef enum
{
    SYNC_JUMP_WIDTH_1X = 0b00000000,
    SYNC_JUMP_WIDTH_2X = 0b01000000,
    SYNC_JUMP_WIDTH_3X = 0b10000000,
    SYNC_JUMP_WIDTH_4X = 0b11000000
}SYNC_JUMP_WIDTH_VALUES;


//! Used for configuration of the CAN peripheral
//! The SEG2PHTS bit controls how the length of
//! Phase Segment 2 is determined.
typedef enum
{
    SEG2PHTS_FREE = 0b10000000,
    SEG2PHTS_CALC = 0b00000000
}SEG2PHTS_VALUES;


//! Used for configuration of the CAN peripheral
//! How many times to sample the bits
typedef enum
{
    SAMPLE_ONCE   = 0b00000000,
    SAMPLE_THRICE = 0b01000000
}SAMPLE_TIMES_VALUES;


//! Used for configuration of the CAN peripheral
//! 
typedef enum
{
    CAN_WAKE_ENABLE  = 0b00000000,
    CAN_WAKE_DISABLE = 0b10000000
}CAN_WAKE_VALUES;

//! Used for configuration of the CAN peripheral
//! We're using CAN_MODE_LEGACY for the simple slave
typedef enum
{
    CAN_MODE_LEGACY           = 0b00000000,
    CAN_MODE_ENHANCED_LEGACY  = 0b01000000,
    CAN_MODE_ENHANCED_FIFO    = 0b10000000
}CAN_MODE_VALUES;

//! Used for configuration of the CAN peripheral
//! 
typedef enum
{
    RECEIVE_ALL_MESSAGES      = 0b01100000,
    RECEIVE_EXTENDED_MESSAGES = 0b01000000,
    RECEIVE_STANDARD_MESSAGES = 0b00100000
}RECEIVE_BUFFER_MODE_VALUES;

//! Used for configuration of the CAN peripheral
//! We should use VDD mode
typedef enum
{
    CANTX_DRIVE_VDD           = 0b00100000,
    CANTX_DRIVE_TRI_STATE     = 0b00000000
}CANTX_DRIVE_VALUES;

// The following #defines are used to configure the CAN bus,
// Specifically the speed and timing, among other things
/*
#define BAUD_RATE_PRESCALER                             2
#define SYNC_JUMP_WIDTH                SYNC_JUMP_WIDTH_1X
#define SEG2PHTS                            SEG2PHTS_FREE
#define SAMPLE_TIMES                        SAMPLE_THRICE
#define PHASE_SEGMENT_1                                 3
#define PHASE_SEGMENT_2                                 3
#define PROPAGATION_SEGMENT                             3
#define CAN_MODE                          CAN_MODE_LEGACY
#define RECEIVE_BUFFER_MODE     RECEIVE_STANDARD_MESSAGES
#define CANTX_DRIVE                       CANTX_DRIVE_VDD
*/

#define BAUD_RATE_PRESCALER                             1
#define SYNC_JUMP_WIDTH                SYNC_JUMP_WIDTH_2X
#define SEG2PHTS                            SEG2PHTS_FREE
#define SAMPLE_TIMES                        SAMPLE_THRICE
#define PHASE_SEGMENT_1                                 6
#define PHASE_SEGMENT_2                                 6
#define PROPAGATION_SEGMENT                             7
#define CAN_MODE                          CAN_MODE_LEGACY
#define RECEIVE_BUFFER_MODE     RECEIVE_STANDARD_MESSAGES
#define CANTX_DRIVE                       CANTX_DRIVE_VDD

//! This initializes the ECAN module of the PIC18F
//! It sets up the baud rate, filters, masks and CAN MODE.
//! 
//! @author Yann Sionneau
static void can_init(void)
{

    while ((CANSTAT & 0b11100000) != CAN_CONFIG_MODE)       // Entering CAN Config mode
    {
        CANCON = CAN_CONFIG_MODE;
    }

    BRGCON1 = SYNC_JUMP_WIDTH | (BAUD_RATE_PRESCALER-1);                                        //0x41;
    BRGCON2 = SEG2PHTS | SAMPLE_TIMES | ((PHASE_SEGMENT_1-1)<<3) | (PROPAGATION_SEGMENT-1);     //0x0B;
    BRGCON3 = CAN_WAKE_DISABLE | (PHASE_SEGMENT_2-1);                                           //0x82;


                                                            // Setup the Baud Rate
/*
    //BRGCON1 = 0x41;                                         // These are the data copied from SimpleMotor
    //BRGCON2 = 0x0B;
    //BRGCON3 = 0x82;

    RXB0CON = 0;

    CIOCON = 0b00100000;
    PIE3 = 0;
*/
    
    ECANCON = CAN_MODE;                    
    RXB1CON = RECEIVE_BUFFER_MODE;
    RXB0CON = RECEIVE_BUFFER_MODE;
    CIOCON  = CANTX_DRIVE;

    RXF0SIDH = 0b11000000 | (motor_id << 2);
    RXF0SIDL = 0;
    RXF1SIDH = 0b11000000 | (motor_id << 2);
    RXF1SIDL = 0;
    RXM0SIDH = 0b11111100;  // set mask to 0b11111100000
    RXM0SIDL = 0;

    RXF2SIDH = 0;
    RXF2SIDL = 0;
    RXF3SIDH = 0;
    RXF3SIDL = 0;
    RXF4SIDH = 0;
    RXF4SIDL = 0;
    RXM1SIDH = 0b11111111;
    RXM1SIDL = 0b11100000;

    TRISB &= ~(1 << 2); 

    // Enter Normal Mode
    while ((CANSTAT & 0b11100000) != CAN_NORMAL_MODE)
        CANCON = CAN_NORMAL_MODE;
}

/*
    11 0100 0 0001
F0: 11 0100 0 0000
F1: 11 0100 0 0000
F2: 01 0000 0 1000

M0: 11 1111 0 0000
M1: 00 0011 0 1001
*/

//! This is the "Send Can Message" function.
//! It sends the content of the CanMsgT struct over the CAN bus.
//!
//!
//! @author Yann Sionneau
static void sendCanMsg(void)
{
    TXB0SIDL = ((CanMsgT.messageID << 5) & 0xff);
    TXB0SIDH = ((CanMsgT.messageID >> 3) & 0xff);
    TXB0DLC  = CanMsgT.length;
    TXB0D0   = CanMsgT.d.byte[0];
    TXB0D1   = CanMsgT.d.byte[1];
    TXB0D2   = CanMsgT.d.byte[2];
    TXB0D3   = CanMsgT.d.byte[3];
    TXB0D4   = CanMsgT.d.byte[4];
    TXB0D5   = CanMsgT.d.byte[5];
    TXB0D6   = CanMsgT.d.byte[6];
    TXB0D7   = CanMsgT.d.byte[7];

    TXB0CON |= (1 << TXREQ);        // Now transmit that message
}

//! This function erases the program memory (which is flash)
//! It only erases the user application
//! So it starts erasing after the bootloader and stops before the debugger code
//!
//! @author Yann Sionneau
static void erase_flash(void)
{
    overlay int16u i = 0;

    i = 0x04c0;                         // user application start address
                                        // 0x7dc0 is the start of debugger code

    while (i < 0x7dc0)                  // and we don't want to erase debugger code
    {
        TBLPTR  = i;           
        TBLPTR &= 0xFFFFE0;
        
        EECON1 |=128;                   // point to Flash program memory
        EECON1 &= ~64;                  // access Flash program memory
        EECON1 |= 4;                    // enable write to memory
        EECON1 |= 16;                   // enable Row Erase operation
        INTCON &= ~128;                 // disable interrupts
        EECON2 = 0x55;
        EECON2 = 0xaa;
        EECON1 |= 2;                    // start erase (CPU stall)
        
        while (EECON1 & 2)              // Wait for stuff being written
        {
        }

        INTCON |= 128;
        i += 64;
    }   
}



//! This function reads the program memory (which is FLASH).
//! It reads 8 bytes at the address present in the incoming CAN
//! command and sends it back through CAN too.
//!
//! @author Yann Sionneau
static void read_flash(void)
{
    overlay int8u i = 0;

    TBLPTRL = CanMsgR.d.byte[0];
    TBLPTRH = CanMsgR.d.byte[1];
    TBLPTRU = CanMsgR.d.byte[2];

    while (i < 8)
    {
        _asm
            TBLRDPOSTINC
        _endasm
        CanMsgT.d.byte[i++] = TABLAT;
    }
    CanMsgT.messageID = CanMsgR.messageID | 0x10; // We set the "ACK" bit in the SID
    CanMsgT.length    = 8;
    sendCanMsg();
}



//! This function acknowledges ( most of ) the commands.
//! Which means sending back the same content, same length, same SID but with the "ACK" bit (0x10).
//!
//! @author Yann Sionneau
static void acknowledge_packet(void)
{
    CanMsgT.messageID = CanMsgR.messageID | 0x10;       // ACK bit
    CanMsgT.d.byte[0] = CanMsgR.d.byte[0];
    CanMsgT.d.byte[1] = CanMsgR.d.byte[1];
    CanMsgT.d.byte[2] = CanMsgR.d.byte[2];
    CanMsgT.d.byte[3] = CanMsgR.d.byte[3];
    CanMsgT.d.byte[4] = CanMsgR.d.byte[4];
    CanMsgT.d.byte[5] = CanMsgR.d.byte[5];
    CanMsgT.d.byte[6] = CanMsgR.d.byte[6];
    CanMsgT.d.byte[7] = CanMsgR.d.byte[7];
    CanMsgT.length    = CanMsgR.length;
    sendCanMsg();
}



//! This is setting up the flash addressing registers to some address received by a CAN command.
//! This sets everything up for future writes.
//!
//! @author Yann Sionneau
static void write_flash_address(void)
{
    position = 0; // resets the position to 0, we are starting to write a 32 bytes block
    TBLPTRU = CanMsgR.d.byte[2];
    TBLPTRH = CanMsgR.d.byte[1];
    TBLPTRL = CanMsgR.d.byte[0];
}



//! This function does the actual writting in program memory (which is FLASH).
//! Writting the flash has to be done by 32 bytes blocks. But we can only transport
//! 8 bytes of data in a CAN message, so we do the block writting in 4 CAN commands.
//! So this command writes 8 bytes, and is called several times (4 times in theory)
//! Each time it is called the "position" variable gets added 8.
//! When position gets equal to 32, it means we have buffered a block and we can start
//! the writting procedure.
//!
//! @author Yann Sionneau
static void write_flash_data(void)
{
    int8u i;
    
    for (i=0 ;i<8; ++i)                                 // Put 8 more bytes in the buffer
    {
        TABLAT = CanMsgR.d.byte[i];
        _asm
            TBLWTPOSTINC
        _endasm
    }

    position += 8;                                      // We just buffered 8 more bytes

    if (position == 32)                                 // We have buffured a 32 bytes block, we can start the writting procedure
    {
        _asm
            TBLRDPOSTDEC                                // We do a TBLRDPOSTDEC in order for the TBLPTR addressing register to stay in the range of the 32 bytes
        _endasm                                         // block we are writting, this is necessary because there has been one extra unneeded TBLWTPOSTINC during
                                                        // the previous loop. If we don't do that, the block will be written over the next block, so 32 bytes after
                                                        // the address provided by the previous WRITE_FLASH_ADDRESS command.

        EECON1 |= ((1 << EEPGD) | (1 << WREN));         // point to Flash program memory & enable write to memory
        EECON1 &= ~(1 << CFGS);                         // access Flash program memory
        INTCON &= ~(1 << GIE);                          // disable interrupts
        EECON2 = 0x55;                                  // magic enable
        EECON2 = 0xaa;
        EECON1 |= (1 << WR);                            // starts the actual writting (CPU stall)
        INTCON |= (1 << GIE);                           // re-enable interrupts
        EECON1 &= ~(1 << WREN);                         // disable write to memory
        position = 0;                                   // reset the position to 0, we just finished a 32 bytes block
    }
} 

//! This function writes to the EEPROM memory
//! The only goal of this function is to write something different than 0xFF
//! at the last memory location in EEPROM. This happens right after a flashing
//! operation has been successfull, to make sure that at the following reboot of the
//! PIC18F, it will boot directly to the user application and not stay in bootloader mode.
//!
//! This function writes 0x42 to the last byte (address 0xFF).
//!
//! @author Yann Sionneau
static void write_eeprom(void)
{
    EEADR   = 0xff;                                     // EEPROM address to write to 
    EEDATA  = 0x42;                                     // Something != than 0xFF it's the data to be written
    EECON1 &= ~( (1 << EEPGD) | (1 << CFGS) );          // we select EEPROM memory
    EECON1 |=    (1 << WREN);                           // enable write to memory
    INTCON &= ~  (1 << GIE );                           // disable interrupts

    EECON2  = 0x55;                                     // magic enable
    EECON2  = 0xaa;
    EECON1 |= (1 << WR);                                // do the write

    while (EECON1 & (1 << WR));                         // wait for the write to finish

    INTCON |=  (1 << GIE );                             // enable back interrupts
    EECON1 &= ~(1 << WREN);                             // disable write to memory
    
}



//! This function first tests if we received a CAN message.
//! If not it does nothing.
//! If we received a CAN message, it fills the CanMsgR struct with it's data
//! And then frees up the reception slot, and then executes the command which
//! is inside the CAN message.
//!
//! @author Yann Sionneau
static void handle_can_msg(void)
{
    if ( (RXB0CON & (1 << RXFUL) ) )                    // This checks if we received a CAN message
    {                                                   // We just received a CAN message, we put it in our CanMsgR reception struct
        CanMsgR.messageID   = RXB0SIDH;
        CanMsgR.messageID <<= 3;
        CanMsgR.messageID  |= RXB0SIDL >> 5;
        CanMsgR.length      = RXB0DLC;
        CanMsgR.d.byte[0]   = RXB0D0;
        CanMsgR.d.byte[1]   = RXB0D1;
        CanMsgR.d.byte[2]   = RXB0D2;
        CanMsgR.d.byte[3]   = RXB0D3;
        CanMsgR.d.byte[4]   = RXB0D4;
        CanMsgR.d.byte[5]   = RXB0D5;
        CanMsgR.d.byte[6]   = RXB0D6;
        CanMsgR.d.byte[7]   = RXB0D7;
        RXB0CON            &= ~(1 << RXFUL);            // We ACK the reception, mark the slot as being empty, free for the next packet
    
        switch (CanMsgR.messageID & 0x000F)             // The command is the last 4 bits of SID
        {
            case WRITE_FLASH_ADDRESS_COMMAND:           // sets up the address for program memory writting
                write_flash_address();
                acknowledge_packet();
                break;

            case WRITE_FLASH_DATA_COMMAND:              // writes to the program memory
                write_flash_data();
                acknowledge_packet();
                break;

            case READ_FLASH_COMMAND:
                read_flash();                           // special ack, done in the function, will ack with the data
                break;

            case ERASE_FLASH_COMMAND:
                erase_flash();                          // erases program memory
                acknowledge_packet();
                break;

            case RESET_COMMAND:
                write_eeprom();
                acknowledge_packet();
                _asm
                    goto 0x0000                     // This jumps to the Reset vector
                _endasm
                // no break needed because of the goto above

            case READ_VERSION_COMMAND: // should return the boot loader version number
                CanMsgT.messageID = CanMsgR.messageID | 0x10;
                CanMsgT.d.byte[0] = 0x40;
                CanMsgT.d.byte[1] = 0x41;
                CanMsgT.d.byte[2] = 0x42;
                CanMsgT.d.byte[3] = 0x43;
                CanMsgT.length = 4;
                sendCanMsg();
                break;

            case MAGIC_PACKET: // Means "reboot", it will be ACK'ed at boot time (in the main())
                _asm
                    goto 0x0000
                _endasm
                // no break needed because of the goto above

            //default:
                // Perhaps we should send back some kind of error packet?       
        }
    }

    if ( (RXB1CON & (1 << RXFUL) ) )
    {
        Nop();
    }
}

void serious_error(void)
{
    while(1)
    {
    }
}

//! Main function
//! This is always executed at startup of the PIC18F regardless of whether there is
//! a user application or not.
//!
//! @author Yann Sionneau
void main(void)
{
    overlay int8u i = 0;
    
    WDTCON = 0;                             // disables the watchdog
                                            // has no effect if the configuration bit WDTEN is enabled


    motor_id = read_eeprom(0x00);

    if (motor_id == 0xFF)
        serious_error();

    if ( we_should_boot() )                 // We check if we should directly jump to the user application
    {
    _asm
        goto 0x04c0                         // This jumps to the _entry of user application
    _endasm
    }


                                            // From here we are in boot loader mode
                                            // ------------------------------------


    position = 0;                           // Resets the position for the "write to flash" process
    can_init();                             // Initializes ECAN module

    CanMsgT.length = 8;                     // The following lines craft an ACK for a magic packet


    //CanMsgT.messageID = 0x600 | (motor_id << 5) | 0x0010 | 0x00A;     // Don't do it this way! (motor_id << 5) looses the top bit.

    CanMsgT.messageID = motor_id;                                       // Do it this way. CanMsgT.messageID is 16 bits.
    CanMsgT.messageID <<= 5;
    CanMsgT.messageID  |= 0x600 | 0x0010 | 0x00A;

    for (i = 0 ; i < 4 ; ++i)
        CanMsgT.d.word[i] = 0xAA55;         // swapped because of endianness

    sendCanMsg();                           // We ACK the magic packet at boot time


    while ( 1 )                             // Main loop
    {
        handle_can_msg();                   // checks if we have received a CAN msg, and handles it
    }

}
