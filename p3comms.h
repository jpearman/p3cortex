/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     p3comms.h                                                    */
/*    Author:     James Pearman                                                */
/*    Created:    13 August 2013                                               */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00  13 Aug 2013 - Initial release                         */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program , however, it is requested that    */
/*    improvements or additions be shared with the Vex community via the vex   */
/*    forum.  Please acknowledge the work of the authors when appropriate.     */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Description:                                                             */
/*    Header for p3comms.c                                                     */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#ifndef P3COMMS_H_
#define P3COMMS_H_

// function return values
#define P3_SUCCESS      0
#define P3_FAILURE      (-1)

// This can be 1 or 2 on the cortex
#define P3_MAX_PORTS    1

// Most messages are 29 bytes of data to save memory
#define P3_SMALL_MSG    (32-3)
// full messages limited in this implementation to save memory
// theoretically we could have up to 255 bytes of payload data but
// most commands will never use that.  This version for ROBOTC is
// more constrained and just an example so we limit to 125 bytes
#define P3_FULL_MSG     (128-3)

// Structure to hold p3 command limited to P3_SMALL_MSG bytes of data
// (P3_SMALL_MSG+3) bytes total
// this is enough for most typical commands
// use p3cmdfull for larger ammounts of data

typedef struct _p3cmd {
    unsigned char   cmd1;
    unsigned char   cmd2;
    unsigned char   length;
    unsigned char   data[P3_SMALL_MSG];
    } p3cmd;

// Structure to hold p3 command limited to P3_FULL_MSG bytes of data
// (P3_FULL_MSG+3) bytes total

typedef struct _p3cmdfull {
    unsigned char   cmd1;
    unsigned char   cmd2;
    unsigned char   length;
    unsigned char   data[P3_FULL_MSG];
    } p3cmdfull;

// ROBOTC does not support nested structures so I had to break these out
typedef struct __cmdpak {
    unsigned char   preamble1;
    unsigned char   preamble2;
    p3cmdfull       cmd;
    } _cmdpak;

typedef union __command {
    _cmdpak         cmdpak;
    unsigned char   data[sizeof(_cmdpak)];
    } _command;

// Structure to hold transmit or receive packet
typedef struct _p3packet {
    _command        command;
    short           cmd_cnt;
    short           cmd_len;
    unsigned char   chk_sum;
    unsigned char   dev_id;
	unsigned char   masked_cmd1;
    } p3pak;

#define P3_BAUD                     baudRate115200

#define P3_RX_BUF_SIZE              256
#define P3_RX_NO_DATA               -1
#define P3_RX_BUF_ERR               -2

// Fixed preambles for the P3 messages
#define P3_PREAMBLE1                0x50
#define P3_PREAMBLE2                0xAF

#define MANUFACTURER_STRING_LEN     32
#define PRODUC_TNAME_STRING_LEN     32
#define SERIAL_NUMBER_STRING_LEN    32

// Defined command 1 groups (4 bits max)
#define CMD1_GROUP_SYSTEM_CMD       0
#define CMD1_GROUP_SYSTEM_REPLY     1
#define CMD1_GROUP_CONTROL          2
#define CMD1_GROUP_PRESET           4
#define CMD1_GROUP_STATUS           6
#define CMD1_GROUP_STATUS_REPLY     7
#define CMD1_GROUP_FACTORY          0x0F

// Defined command 2 commands
#define CMD2_SYSTEM_ACK             0x10
#define CMD2_SYSTEM_DEVICE_TYPE     0x11
#define CMD2_SYSTEM_NAK             0x12
#define CMD2_SYSTEM_MANUFACTURER    0x13
#define CMD2_SYSTEM_PRODUCT_NAME    0x14
#define CMD2_SYSTEM_SERIAL_NUM      0x15

#define CMD2_SYSTEM_FIRMWARE        0x20
#define CMD2_SYSTEM_HARDWARE        0x21

#define CORTEX_DEVICE_ID            0x00
#define GLOBAL_DEVICE_ID            0x0F

// mode determines whether we are running as a master (host) or slave (client)
typedef enum  {
    kP3ModeSlave = 0,
    kP3ModeMaster
    } p3mode;

// communications phase
typedef enum  {
    kP3StateIdle = 0,
    kP3StateCommandWait,
    kP3StateReplyWait,
    kP3StateReplyWaitTxPend,
    kP3StateTimeout
    } p3state;

// A structure to collect all information together for a single
// communicatuoibs channel
typedef struct _p3comms {
    int             port;       // which Uart
	long            baud;       // baud rate of channel
    p3mode          mode;       // master or slave
    p3state         state;      // communications state
    int             tcount;     // number of timeouts

    int             online;     // status of slave (master mode only)

    // Transmit and Receive packets
    p3pak           TxPak;
    p3pak           RxPak;
    p3pak           ExPak;

    // Receive buffer
    int             rxcnt;                      // last amount of rx data
    unsigned char   rxbuf[P3_RX_BUF_SIZE];      // buffer for rx data
    int             rxto;                       // timeout counter

    // debug
    int             debug;
    int             DebugTx;    // display transmit packets on console
    int             DebugRx;    // display receive packets on console

    // storage to help with checking rx buffer
    int             peekChar;

	// standard reply storage
	unsigned char  deviceType[2];
	unsigned char  manufacturer[MANUFACTURER_STRING_LEN];
	unsigned char  product_name[PRODUC_TNAME_STRING_LEN];
    unsigned char  serial_number[SERIAL_NUMBER_STRING_LEN];
	unsigned char  firmware_version[5];
	unsigned char  hardware_version[3];
} p3comms;


// Prototypes
p3comms *   P3Init(int port, p3mode mode, int debug_flag, long baud );
void        P3Deinit(p3comms *MyComms);
int         P3InitSerial(p3comms *MyComms);
int         P3Command( p3comms *MyComms, void *command, int dest_id  );
void        P3DebugPacket( p3pak *packet );
int         P3SendPacket( p3comms *MyComms, p3pak *packet );
int         P3ReceiveData( p3comms *MyComms);
void        P3ReceivePacket( p3comms *MyComms );
void        P3DecodePacket( p3comms *MyComms, p3pak *packet );
void        P3DecodeSysCtl( p3comms *MyComms, p3pak *packet );
void        P3DecodeSysReply( p3comms *MyComms, p3pak *packet );
int         P3CommsTask( p3comms *MyComms );

void        P3SetManufacturerString( p3comms *MyComms, char *str );
void        P3SetProductNameString( p3comms *MyComms, char *str );
void        P3SetSerialNumberString( p3comms *MyComms, char *str );
void        P3SetFirmwareVersion( p3comms *MyComms, unsigned char major, unsigned char minor, unsigned char bug, unsigned short build );
void        P3SetHardwareVersion( p3comms *MyComms, unsigned char major, unsigned char minor, unsigned char revision );

// There are no function callbacks in ROBOTC, user must provide this function
int         P3UserDecodePacket( p3comms *MyComms, p3pak *packet );

#endif /* P3COMMS_H_ */
