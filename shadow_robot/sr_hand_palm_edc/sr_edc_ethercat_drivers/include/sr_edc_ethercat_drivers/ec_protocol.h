/* ec_protocol.h */
/* Documents all of the data structures that we pass across EtherCAT*/
/* (C) Shadow Robot Company, 2009 */

#ifndef _EC_PROTOCOL_H
#define _EC_PROTOCOL_H


#define int8 uint8_t
#define int16 uint16_t
#define int16s int16_t
#define int32 uint32_t

/* Base bridge node */
#define EC_PRODUCT_ID_BRIDGE 0

/**************************************************************************** 
   EC_MOTOR node, running "bare" motor driving protocol
   Product ID 1
   */

/* Communication notes:
   Write command to actuator_data.
   Wait a short time.
   Read sensor_data.
   Check last_command_executed == command_number.
*/


#define EC_PRODUCT_ID_MOTOR 1

#define EC_MOTOR_COMMAND_PHY_BASE 0x1020
#define EC_MOTOR_DATA_PHY_BASE 0x1000


struct ec_motor_actuator_data
{
    int16s hbridge_duty;
    int16  hbridge_flags;

    int8   command_number;
    int8   ref0;
    int8   ref1;
    int8   magic;

    int8   ref0_write_to_EEPROM;
    int8   ref1_write_to_EEPROM;
}
//#ifdef GCC
__attribute__((packed))
//#endif
;


struct ec_motor_sensor_data
{
    int16 force_sensor_1, force_sensor_2, force_sensor_3;
    int16 motor_current, motor_voltage, temperature_val;
    int16 station_alias;
    int16 sensor_read_count; //!< Incremented on each update of the values.
    int16s last_hbridge;
    int16 dummy;
    int8 last_command_executed;
    int8 magic;

    int8   SG_ref_0, SG_ref_1;
    int16s SG_zero_0, SG_zero_1;
    int16  SG_precal_0, SG_precal_1;
}
//#ifdef GCC
__attribute__((packed))
//#endif
;



/**************************************************************************** 
   EC_SHADOWCAN node, providing a CAN mapping for one PALM node and power control
   Product ID 2
   */

#define EC_PRODUCT_ID_SHADOWCAN 2

#define EC_SHADOWCAN_COMMAND_PHY_BASE 0x1000
#define EC_SHADOWCAN_SENSOR_PHY_BASE 0x1100

#define EC_SHADOWCAN_IO_STATUS_FAN_ON   1
#define EC_SHADOWCAN_IO_STATUS_MOTOR_ON 2
#define EC_SHADOWCAN_IO_STATUS_NODE_ON  4
#define EC_SHADOWCAN_IO_STATUS_ECAT_ON  8

struct ec_shadowcan_sensor_data {
  int16 last_command_number;
  int32 rx_message_count;
  int16s dat[36];
  int8 io_status; // bits set in this are EC_SHADOWCAN_IO_STATUS bits
}
//#ifdef GCC
__attribute__((packed))
//#endif
;

enum ec_shadowcan_commands { 
  EC_SHADOWCAN_NODE_ON=1,
  EC_SHADOWCAN_NODE_OFF=2,
  EC_SHADOWCAN_MOTOR_ON=3,
  EC_SHADOWCAN_MOTOR_OFF=4,
  EC_SHADOWCAN_ECAT_ON=5,
  EC_SHADOWCAN_ECAT_OFF=6,
};

#define EC_SHADOWCAN_COMMAND_MAGIC 0xFFFF

struct ec_shadowcan_command_data {
  int16 command_number; // enum ec_shadowcan_commands
  int16 command_magic; // Must be EC_SHADOWCAN_COMMAND_MAGIC
}
//#ifdef GCC
__attribute__((packed))
//#endif
;

/**************************************************************************** 
   EC_DUALMOTOR node, running two motors
   Product ID 3
   */

/* Communication notes:
   Write command to actuator_data[X].
   Wait a short time.
   Read sensor_data[X].
   Check last_command_executed == command_number.
*/


#define EC_PRODUCT_ID_DUALMOTOR 3

#define EC_DUALMOTOR_COMMAND_PHY_BASE 0x1040
#define EC_DUALMOTOR_DATA_PHY_BASE 0x1000

struct ec_dualmotor_actuator_data {
  struct ec_motor_actuator_data m[2];
}   
//#ifdef GCC
__attribute__((packed))
//#endif
;
struct ec_dualmotor_sensor_data {
  struct ec_motor_sensor_data s[2];
}
//#ifdef GCC
__attribute__((packed))
//#endif
;

/* Defined for naming actuators on ShadowHand */

#define NUM_OF_ACTUATORS 20

const char *SR_ACTUATOR_NAME[NUM_OF_ACTUATORS + 2] = { /* + 2 is just for placing
                                                   bridge and can controller 
                                                   name which are not really
                                                   an actuator */
    "sr_bridge", /* Not an actuator */
    "sr_cancontroller", /* Not exactly an actuator, we will be using this 
                           internally for merging other sensor's data with 
                           position sensor data from this controller */
    "sr_dmotor_ffj0", // 2
    "sr_dmotor_ffj3", // 3
    "sr_dmotor_ffj4", // 4

    "sr_dmotor_mfj0", // 5
    "sr_dmotor_mfj3", // 6
    "sr_dmotor_mfj4", // 7
    
    "sr_dmotor_rfj0", // 8
    "sr_dmotor_rfj3", // 9
    "sr_dmotor_rfj4", // 10
    
    "sr_dmotor_lfj0", // 11
    "sr_dmotor_lfj3", // 12
    "sr_dmotor_lfj4", // 13
    "sr_dmotor_lfj5", // 14
    
    "sr_dmotor_thj1", // 15
    "sr_dmotor_thj2", // 16
    "sr_dmotor_thj3", // 17
    "sr_dmotor_thj4", // 18
    "sr_dmotor_thj5", // 19
    
    "sr_dmotor_wrj1", // 20
    "sr_dmotor_wrj2"  // 21
};

struct SR_Actuator_Mappings {
    uint8_t name_ref; /* number for name entry in above char array */
    uint8_t slave_no; /* ET1200 slave number */
    uint8_t slave_offset; /* number for finding Motor 0 or 1 attached to slave */
    uint8_t can_ctrl_offset; /* Offset in can controller position data array */
    uint8_t can_ctrl_offset1; /* Offset in can controller position data array 
                                 in case of compound joint */
};

#define SR_MOTOR_START_SLAVE_NUM 33

/* FIXME: Have to check slave_offset when testing that which is 0 and which is 1 */
/* FIXME: Have to check can_ctrl_offset when testing that is the current value is valid or not */
/* FIXME: These positions are considering only Shadow Hand connected to system, hence will
 * not be valid for compelete PR2 robot. Need fixing when reach at that stage of merging. */
const struct SR_Actuator_Mappings SR_ACTUATOR_MAP[NUM_OF_ACTUATORS + 2] = {
    {-1, -1, -1, -1, -1},
    {-1, -1, -1, -1, -1},

    /* FFJ0 = FFJ1 + FFJ2; so we are filling both can_control_offsets */
    //    {2, 4, 0, 0, 1},

    {6, 33, 0, 6, -1}, //mfj3: motor 1
    {5, 33, 1, 25, -1}, //mfj0: motor 2
    {7, 34, 0, 7, -1}, //mfj4: motor 3
    {8, 34, 1, 26, -1}, //rfj0: motor 4

    {10, 35, 0, 11, -1}, //rfj4: motor 5
    {9,  35, 1, 10, -1}, //rfj3: motor 6
    {15, 36, 0, 17, -1}, //thj1: motor 7
    {12, 36, 1, 14, -1}, //lfj3: motor 8
 
    {11, 37, 0, 27, -1}, //lfj0: motor 9
    {13, 37, 1, 15, -1}, //lfj4: motor 10
    {16, 38, 0, 18, -1}, //thj2: motor 11
    {14, 38, 1, 16, -1}, //lfj5: motor 12
 
    {2, 39, 0, 24, -1}, //ffj0: motor 13
    {3, 39, 1, 2, -1}, //ffj3: motor 14
    {4, 40, 0, 3, -1}, //ffj4: motor 15
    {17, 40, 1, 19, -1}, //thj3: motor 16

    {21, 41, 0, 23, -1}, //wrj2: motor 18
    {20, 41, 1, 22, -1}, //wrj1: motor 17

    {19, 42, 0, 21, -1}, //thj5: motor 20
    {18, 42, 1, 20, -1} //thj4: motor 19
};

/* Below stuff is just for reference */
//    sensor FFJ1_Pos is palm_sensor.0 type none
//    sensor FFJ2_Pos is palm_sensor.1 type none
//    sensor FFJ3_Pos is palm_sensor.2 type none
//    sensor FFJ4_Pos is palm_sensor.3 type none
//
//    sensor MFJ1_Pos is palm_sensor.4 type none 
//    sensor MFJ2_Pos is palm_sensor.5 type none
//    sensor MFJ3_Pos is palm_sensor.6 type none
//    sensor MFJ4_Pos is palm_sensor.7 type none 
//
//    sensor RFJ1_Pos is palm_sensor.8 type none
//    sensor RFJ2_Pos is palm_sensor.9 type none
//    sensor RFJ3_Pos is palm_sensor.10 type none
//    sensor RFJ4_Pos is palm_sensor.11 type none 
//
//    sensor LFJ1_Pos is palm_sensor.12 type none 
//    sensor LFJ2_Pos is palm_sensor.13 type none
//    sensor LFJ3_Pos is palm_sensor.14 type none
//    sensor LFJ4_Pos is palm_sensor.15 type none 
//    sensor LFJ5_Pos is palm_sensor.17 type none
//
//    sensor THJ1_Pos is palm_sensor.18 type none 
//    sensor THJ2_Pos is palm_sensor.19 type none 
//    sensor THJ3_Pos is palm_sensor.20 type none
//    sensor THJ4_Pos is palm_sensor.21 type none
//
//    # Don't do these two - they are actually calibrated!
//    sensor THJ5_Pos is palm_sensor.44 type none 
//    sensor WRJ1_Pos is palm_sensor.45 type none
//
//    sensor WRJ2_Pos is palm_sensor.22 type none 

#define HBRIDGE_FLAG_BRAKE_MASK     1
#define HBRIDGE_FLAG_BRAKE_BRAKE    1
#define HBRIDGE_FLAG_BRAKE_NOBRAKE  0

#define HBRIDGE_FLAG_SLEEP_MASK     2
#define HBRIDGE_FLAG_SLEEP_WAKE     2
#define HBRIDGE_FLAG_SLEEP_SLEEP    0

#define HBRIDGE_MAX_DUTY            0x3FF
#define HBRIDGE_SLEEP_TIME          0x80            //<! How long before hbridge_duty==0 causes
                                                    //<! me to put the hbridge into sleep mode. 0xFF max

/* SmartMotor/hbridge.h stuff end */

//static const struct { const char * name; const char *sub; int num; } hand_joint_list[] = {
//
//  [HAND_FF0]={"FF", "0", 1},
//  [HAND_FF1]={"FF", "1", 0},
//  [HAND_FF2]={"FF", "2", 0},
//  [HAND_FF3]={"FF", "3", 1},
//  [HAND_FF4]={"FF", "4", 1},
//  
//  [HAND_LF0]={"LF", "0", 1},
//  [HAND_LF1]={"LF", "1", 0},
//  [HAND_LF2]={"LF", "2", 0},
//  [HAND_LF3]={"LF", "3", 1},
//  [HAND_LF4]={"LF", "4", 1},
//  [HAND_LF5]={"LF", "5", 1},
//  
//  [HAND_MF0]={"MF", "0", 1},
//  [HAND_MF1]={"MF", "1", 0},
//  [HAND_MF2]={"MF", "2", 0},
//  [HAND_MF3]={"MF", "3", 1},
//  [HAND_MF4]={"MF", "4", 1},
//  
//  [HAND_RF0]={"RF", "0", 1},
//  [HAND_RF1]={"RF", "1", 0},
//  [HAND_RF2]={"RF", "2", 0},
//  [HAND_RF3]={"RF", "3", 1},
//  [HAND_RF4]={"RF", "4", 1},
//  
//  [HAND_TH1]={"TH", "1", 1},
//  [HAND_TH2]={"TH", "2", 1},
//  [HAND_TH3]={"TH", "3", 1},
//  [HAND_TH4]={"TH", "4", 1},
//  [HAND_TH5]={"TH", "5", 1},
//  [HAND_WR1]={"WR", "1", 1},
//  [HAND_WR2]={"WR", "2", 1},
//  
//  [NUM_HAND_JOINTS]={0, 0, 0}
//};
//
//static const struct { const float min; const float max; } hand_joint_range[] = {
//	[HAND_LF0]={0,180},
//	[HAND_LF3]={0,90},
//	[HAND_LF4]={-25,25},
//	[HAND_LF5]={0,40},
//	[HAND_RF0]={0,180},
//	[HAND_RF3]={0,90},
//	[HAND_RF4]={-25,25},
//	[HAND_MF0]={0,180},
//	[HAND_MF3]={0,90},
//	[HAND_MF4]={-25,25},
//	[HAND_FF0]={0,180},
//	[HAND_FF3]={0,90},
//	[HAND_FF4]={-25,25},
//	[HAND_TH1]={0,90},
//	[HAND_TH2]={-30,30},
//	[HAND_TH3]={-15,15},
//	[HAND_TH4]={0,75},
//	[HAND_TH5]={-60,60},
//	[HAND_WR1]={-45,35},
//	[HAND_WR2]={-30,10},
//
//	[NUM_HAND_JOINTS]={0, 0}
//};


#endif /* _EC_PROTOCOL_H */

