/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) James Pearman                          */
/*                                   2013                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     p3comms.c                                                    */
/*    Author:     James Pearman                                                */
/*    Created:    4 Oct 2011                                                   */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00     5 Sept 2013 - Initial release for ConVEX & PROS    */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    This file is part of ConVEX.                                             */
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. ConVEX is free software; you can redistribute it         */
/*    and/or modify it under the terms of the GNU General Public License       */
/*    as published by the Free Software Foundation; either version 3 of        */
/*    the License, or (at your option) any later version.                      */
/*                                                                             */
/*    ConVEX is distributed in the hope that it will be useful,                */
/*    but WITHOUT ANY WARRANTY; without even the implied warranty of           */
/*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            */
/*    GNU General Public License for more details.                             */
/*                                                                             */
/*    You should have received a copy of the GNU General Public License        */
/*    along with this program.  If not, see <http://www.gnu.org/licenses/>.    */
/*                                                                             */
/*    A special exception to the GPL can be applied should you wish to         */
/*    distribute a combined work that includes ConVEX, without being obliged   */
/*    to provide the source code for any proprietary components.               */
/*    See the file exception.txt for full details of how and when the          */
/*    exception can be applied.                                                */
/*                                                                             */
/*    The author can be contacted on the vex forums as jpearman                */
/*    or electronic mail using jbpearman_at_mac_dot_com                        */
/*    Mentor for team 8888 RoboLancers, Pasadena CA.                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Description:                                                             */
/*    A port of my p3comms code for the VEX cortex                             */
/*    provides a host/client binary message system similar to the Sony P2      */
/*    protocol but with extensions to allow multidrop communications as well   */
/*    as increased message length                                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
#define   _TARGET_PROS_   1
/*-----------------------------------------------------------------------------*/

#include "string.h"

#ifdef  _TARGET_CONVEX_
#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "vex.h"        // vex library header
#else
#include "main.h"       // vex library header
#endif

#include "p3comms.h"    // p3comms header

// Standard system replies
//static  p3cmd   Cmd_Ack                     = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_ACK,          0x01, {0x00} };
static  p3cmd   Cmd_Nak_Und                 = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_NAK,          0x01, {0x01} };
static  p3cmd   Cmd_Nak_Chksum              = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_NAK,          0x01, {0x04} };
//static  p3cmd   Cmd_Nak_Para_Err            = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_NAK,          0x01, {0x08} };
static  p3cmd   Cmd_Nak_Timeout             = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_NAK,          0x01, {0x80} };

static  p3cmd   Cmd_Dev_Type_Reply          = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_DEVICE_TYPE,  0x02, {0x22, 0xC0} };
static  p3cmd   Cmd_Manufacturer_Reply      = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_MANUFACTURER, 0x00, {0x00} };
static  p3cmd   Cmd_ProductName_Reply       = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_PRODUCT_NAME, 0x00, {0x00} };
static  p3cmd   Cmd_SerialNumber_Reply      = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_SERIAL_NUM,   0x00, {0x00} };
static  p3cmd   Cmd_FirmwareVersion_Reply   = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_FIRMWARE,     0x05, {0x00, 0x00, 0x00, 0x00, 0x00} };
static  p3cmd   Cmd_HardwareVersion_Reply   = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_HARDWARE,     0x03, {0x00, 0x00, 0x00} };

// System commands
//static  p3cmd   Cmd_Dev_Type                = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_DEVICE_TYPE,  0, {0x00} };
//static  p3cmd   Cmd_Manufacturer_Request    = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_MANUFACTURER, 0, {0x00} };
//static  p3cmd   Cmd_ProductName_Request     = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_PRODUCT_NAME, 0, {0x00} };
//static  p3cmd   Cmd_SerialNumber_Request    = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_SERIAL_NUM,   0, {0x00} };
//static  p3cmd   Cmd_FirmwareRev_Request     = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_FIRMWARE,     0, {0x00} };
//static  p3cmd   Cmd_HardwareRev_Request     = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_HARDWARE,     0, {0x00} };


/*---------------------------------------------------------------------------*/
/*  ConVEX glue code                                                         */
/*---------------------------------------------------------------------------*/

#ifdef  _TARGET_CONVEX_

static int
serial_init( p3comms *MyComms, long baud )
{
    SerialConfig config =
        {
        baud,
        USART_CR1_PS | USART_CR1_PCE | USART_CR1_M, // odd parity
        USART_CR2_STOP1_BITS,
        0
        };

    if( MyComms->sdp != NULL )
        {
        // no need to start console serial driver if that port was chosen
        if( MyComms->sdp != SD_CONSOLE )
            sdStart( MyComms->sdp, &config);
        return(1);
        }
    return(0);
}

/*---------------------------------------------------------------------------*/
/*  Peek to see if there are characters in the receive FIFO                  */
/*---------------------------------------------------------------------------*/

static int
serial_peekinput(p3comms *MyComms)
{
    // if we would block then no chars        
    if( sdGetWouldBlock( (SerialDriver *)MyComms->sdp) )
        return(-1);
    else
        return(0);
}

/*---------------------------------------------------------------------------*/
/*  Get next character from the receive FIFO                                 */
/*---------------------------------------------------------------------------*/

static int
serial_getchar(p3comms *MyComms)
{
    int c;

    // get character
    c = sdGetTimeout( (SerialDriver *)MyComms->sdp, TIME_IMMEDIATE);
    if( c != Q_TIMEOUT && c != Q_RESET )
        return(c);
    else
        return(-1);
}

/*---------------------------------------------------------------------------*/
/*  Write buffer to the uart                                                 */
/*---------------------------------------------------------------------------*/

static int
serial_writebuf( p3comms *MyComms, unsigned char *data, int data_len )
{   
    sdWrite( (SerialDriver *)MyComms->sdp, data, data_len);

    return(0);
}

#else
/*---------------------------------------------------------------------------*/
/*  PROS glue code                                                           */
/*---------------------------------------------------------------------------*/

static int
serial_init( p3comms *MyComms, long baud )
{
    if( MyComms->sdp != NULL )
        {
        usartInit( MyComms->sdp, baud, SERIAL_DATABITS_9 | SERIAL_PARITY_ODD);
        return(1);
        }
    return(0);
}

/*---------------------------------------------------------------------------*/
/*  Peek to see if there are characters in the receive FIFO                  */
/*---------------------------------------------------------------------------*/

static int
serial_peekinput(p3comms *MyComms)
{
    // get number of chars available     
    if( fcount( MyComms->sdp ) <= 0 )
        return(-1);
    else
        return(0);
}

/*---------------------------------------------------------------------------*/
/*  Get next character from the receive FIFO                                 */
/*---------------------------------------------------------------------------*/

static int
serial_getchar(p3comms *MyComms)
{
    int c;

    // get character
    if( serial_peekinput( MyComms) >= 0 )
        {
        c = fgetc( MyComms->sdp ) & 0xFF;
        return(c);
        }
    else
        return(-1);
}

/*---------------------------------------------------------------------------*/
/*  Write buffer to the uart                                                 */
/*---------------------------------------------------------------------------*/

static int
serial_writebuf( p3comms *MyComms, unsigned char *data, int data_len )
{   
    int i;
    unsigned char *p = data;
    
    for(i=0;i<data_len;i++)
        fputc( *p++, MyComms->sdp );

    return(0);
}

#endif

/*---------------------------------------------------------------------------*/
/*  Init                                                                     */
/*---------------------------------------------------------------------------*/

p3comms *
P3Init( int port, p3mode mode, int debug_flag, long baud )
{
    p3comms *MyComms = NULL;
    int     i;

    // Allocate a new communication instance
#ifdef  _TARGET_CONVEX_
    MyComms = (p3comms *)chHeapAlloc( NULL, sizeof(p3comms) );
#else
    MyComms = (p3comms *)malloc( sizeof(p3comms) );
#endif

    if(MyComms != NULL)
        {
        // clear memory as we have allocated and may be random data
        memset( MyComms, 0, sizeof(p3comms));

        // Note which serial port we are using
        MyComms->port    = port;
        MyComms->baud    = baud;
        MyComms->mode    = mode;
        MyComms->state   = kP3StateIdle;

        MyComms->debug   = debug_flag;

        // set off by default
        MyComms->DebugRx = 0;
        MyComms->DebugTx = 0;

        // clear strings
        for(i=0;i<MANUFACTURER_STRING_LEN;i++)
            MyComms->manufacturer[i] = 0;
        for(i=0;i<PRODUC_TNAME_STRING_LEN;i++)
            MyComms->product_name[i] = 0;
        for(i=0;i<SERIAL_NUMBER_STRING_LEN;i++)
            MyComms->serial_number[i] = 0;

#ifdef  _TARGET_CONVEX_
        // Generally avoid SD1, use SD2 or SD3
        if( port == 0 )
            MyComms->sdp = &SD1;
        else
        if( port == 1 )
            MyComms->sdp = &SD2;
        else
            MyComms->sdp = &SD3;
#else
        if( port == 0 )
            MyComms->sdp = stdout;
        else
        if( port == 1 )
            MyComms->sdp = uart1;
        else
            MyComms->sdp = uart2;
#endif

        // Callbacks
        MyComms->init       = serial_init;
        MyComms->deinit     = NULL;
        MyComms->flush      = NULL;
        MyComms->write_buf  = serial_writebuf;
        MyComms->peek_input = serial_peekinput;
        MyComms->get_byte   = serial_getchar;

        // Init the serial port here
        P3InitSerial( MyComms );
        }

    return( MyComms );
}

/*---------------------------------------------------------------------------*/
/*      Close the P3 communications                                          */
/*---------------------------------------------------------------------------*/

void
P3Deinit(p3comms *MyComms)
{
    if(MyComms != NULL)
        {
        // Deinit serial port
        if(MyComms->deinit != NULL)
            MyComms->deinit( MyComms );

#ifdef  _TARGET_CONVEX_
        chHeapFree(MyComms);
#else
        free(MyComms);
#endif    
        }
}

/*---------------------------------------------------------------------------*/
/*      Initialize serial port                                               */
/*---------------------------------------------------------------------------*/

int
P3InitSerial(p3comms *MyComms)
{
    if(MyComms->init != NULL)
        MyComms->init( MyComms, MyComms->baud );

    return(P3_SUCCESS);
}

/*---------------------------------------------------------------------------*/
/*      Utility - set callback for user packet decode                        */
/*---------------------------------------------------------------------------*/

void
P3SetReplyDecoder( p3comms *MyComms, void *callback )
{
    MyComms->packet_decode = callback;
}

/*---------------------------------------------------------------------------*/
/*      Utility - set manufacturer without using string functions            */
/*---------------------------------------------------------------------------*/
void
P3SetManufacturerString( p3comms *MyComms, char *str )
{
    int     i;
    char    *p = str;

    // copy string
    for(i=0;(i<MANUFACTURER_STRING_LEN-1) && (*p!=0);i++)
        MyComms->manufacturer[i] = *p++;

    // make sure we are null terminated if the string was truncated
    MyComms->manufacturer[i] = 0;
}

/*---------------------------------------------------------------------------*/
/*      Utility - set product name without using string functions            */
/*---------------------------------------------------------------------------*/
void
P3SetProductNameString( p3comms *MyComms, char *str )
{
    int     i;
    char    *p = str;

    // copy string
    for(i=0;(i<PRODUC_TNAME_STRING_LEN-1) && (*p!=0);i++)
        MyComms->product_name[i] = *p++;

    // make sure we are null terminated if the string was truncated
    MyComms->product_name[i] = 0;
}

/*---------------------------------------------------------------------------*/
/*      Utility - set serial number without using string functions           */
/*---------------------------------------------------------------------------*/
void
P3SetSerialNumberString( p3comms *MyComms, char *str )
{
    int     i;
    char    *p = str;

    // copy string
    for(i=0;(i<SERIAL_NUMBER_STRING_LEN-1) && (*p!=0);i++)
        MyComms->serial_number[i] = *p++;

    // make sure we are null terminated if the string was truncated
    MyComms->serial_number[i] = 0;
}

/*---------------------------------------------------------------------------*/
/*      Utility - set firmware version                                       */
/*---------------------------------------------------------------------------*/
void
P3SetFirmwareVersion( p3comms *MyComms, unsigned char major, unsigned char minor, unsigned char bug, unsigned short build )
{
    MyComms->firmware_version[0] = major;
    MyComms->firmware_version[1] = minor;
    MyComms->firmware_version[2] = bug;
    MyComms->firmware_version[3] = (build >> 8) & 0xFF;
    MyComms->firmware_version[4] = (build     ) & 0xFF;
}

/*---------------------------------------------------------------------------*/
/*      Utility - set firmware version                                       */
/*---------------------------------------------------------------------------*/
void
P3SetHardwareVersion( p3comms *MyComms, unsigned char major, unsigned char minor, unsigned char revision )
{
    MyComms->hardware_version[0] = major;
    MyComms->hardware_version[1] = minor;
    MyComms->hardware_version[2] = revision;
}

/*---------------------------------------------------------------------------*/
/*      Take P3 command and place into tx packet                             */
/*---------------------------------------------------------------------------*/

int
P3Command( p3comms *MyComms, void *command, int dest_id )
{
    unsigned int     i;
    unsigned char   *p, *q;
    p3pak           *MyPak;
    p3cmdfull       *MyCmd = (p3cmdfull *)command;

    MyPak = &MyComms->TxPak;

    // Get command length, add 6 bytes for overhead
    MyPak->cmd_len = (MyCmd->length) + 6;

    // Create header
    MyPak->command.data[0] = P3_PREAMBLE1;
    MyPak->command.data[1] = P3_PREAMBLE2;
    MyPak->command.data[2] = (MyCmd->cmd1 << 4) + (dest_id & 0x0F);
    MyPak->command.data[3] = MyCmd->cmd2;
    MyPak->command.data[4] = MyCmd->length;

    // Start of checksum
    q = &MyPak->command.data[0];
    for(i=0,MyPak->chk_sum = 0;i<5;i++)
        MyPak->chk_sum ^= *q++;

    // move any data that exists
    if( MyCmd->length > 0 )
        {
        p = &MyCmd->data[0];
        for(i=0;i<MyCmd->length;i++)
            {
            MyPak->chk_sum ^= *p;
            *q++ = *p++;
            }
        }

    // put checksum into packet
    *q++ = MyPak->chk_sum;

    // Send packet
    if(MyComms->mode == kP3ModeMaster)
        {
        if( MyComms->state != kP3StateReplyWait )
            {
            P3SendPacket( MyComms, MyPak );

            MyComms->state = kP3StateReplyWait;
            }
        else
            {
            MyComms->state = kP3StateReplyWaitTxPend;
            }
        }
    else
        P3SendPacket( MyComms, MyPak );

    return( P3_SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Print a packet for debug purposes                                    */
/*---------------------------------------------------------------------------*/

void
P3DebugPacket( p3pak *packet )
{
    unsigned char   *p;
    int             i;

    p = &packet->command.data[0];
#ifdef  _TARGET_CONVEX_
    for(i=0;i<packet->cmd_len;i++)
        vex_printf("%02X ",*p++);
    vex_printf("\r\n");
#else
    for(i=0;i<packet->cmd_len;i++)
        printf("%02X ",*p++);
    printf("\r\n");
#endif
    return;
}

/*---------------------------------------------------------------------------*/
/*      Take P3 packet and start transmission                                */
/*---------------------------------------------------------------------------*/

int
P3SendPacket( p3comms *MyComms, p3pak *packet )
{
    // If master set timeout to 10mS
    if(MyComms->mode == kP3ModeMaster)
        MyComms->rxto = 5;

    // Debug
    if(MyComms->DebugTx)
        P3DebugPacket( packet );

    // Transmit
    MyComms->write_buf( MyComms, packet->command.data, packet->cmd_len );

    return( P3_SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Check for serial port for some data                                  */
/*---------------------------------------------------------------------------*/

int
P3ReceiveData(p3comms *MyComms)
{
    int         data;

    // No data then return immeadiately
    if( MyComms->peek_input(MyComms) < 0 )
        {
        if( MyComms->rxto > 0 )
            {
            MyComms->rxto--;

            // Interbyte timeout
            if( MyComms->rxto == 0 )
                {
                MyComms->RxPak.cmd_cnt = 0;
                MyComms->state = kP3StateTimeout;
                MyComms->tcount++;

                // if slave then send timeout NAK
                if( MyComms->mode == kP3ModeSlave )
                    P3Command(MyComms, &Cmd_Nak_Timeout , GLOBAL_DEVICE_ID );
                }
            }

        return( P3_RX_NO_DATA );
        }

    // At least one byte available
    MyComms->rxcnt = 0;

    // Read everything available
    do
        {
        data = MyComms->get_byte(MyComms);
        if( data >= 0 )
            {
            MyComms->rxbuf[MyComms->rxcnt++] = data;
            if( MyComms->rxcnt == P3_RX_BUF_SIZE )
                {
                return( P3_RX_BUF_ERR );
                }
            }
        } while( data >= 0 );

    // timeout set to 5 calls, usually 10mS
    MyComms->rxto = 5;

    return( MyComms->rxcnt );
 }

/*---------------------------------------------------------------------------*/
/*      Received some data so start to decode packet                         */
/*---------------------------------------------------------------------------*/

void
P3ReceivePacket( p3comms *MyComms )
{
    int             i;
    p3pak           *RxPak;

    RxPak = &MyComms->RxPak;

    for(i=0;i<MyComms->rxcnt;i++)
        {
        switch( RxPak->cmd_cnt )
            {
            case    0: // should be preamble 1
                if( MyComms->rxbuf[i] == P3_PREAMBLE1 )
                    {
                    // for debug
                    RxPak->command.cmdpak.preamble1 = P3_PREAMBLE1;
                    RxPak->cmd_cnt = 1;
                    }
                else
                    RxPak->cmd_cnt = 0;
                break;

            case    1: // should be preamble 2
                if( MyComms->rxbuf[i] == P3_PREAMBLE2 )
                    {
                    // for debug
                    RxPak->command.cmdpak.preamble2 = P3_PREAMBLE2;
                    RxPak->chk_sum = P3_PREAMBLE1 ^ P3_PREAMBLE2;
                    RxPak->cmd_cnt = 2;
                    }
                else
                    RxPak->cmd_cnt = 0;
                break;

            case    2:
                RxPak->command.cmdpak.cmd.cmd1 = MyComms->rxbuf[i]; // don;t mask now
                RxPak->dev_id  = MyComms->rxbuf[i] & 0x0F;
                RxPak->masked_cmd1 = (MyComms->rxbuf[i] >> 4) & 0x0F;
                RxPak->chk_sum = RxPak->chk_sum ^ MyComms->rxbuf[i];
                RxPak->cmd_cnt++;
                break;

            case    3:
                RxPak->command.cmdpak.cmd.cmd2 = MyComms->rxbuf[i];
                RxPak->chk_sum = RxPak->chk_sum ^ MyComms->rxbuf[i];
                RxPak->cmd_cnt++;
                break;

            case    4:
                RxPak->command.cmdpak.cmd.length = MyComms->rxbuf[i];
                RxPak->cmd_len = MyComms->rxbuf[i] + 6;
                RxPak->chk_sum = RxPak->chk_sum ^ MyComms->rxbuf[i];
                RxPak->cmd_cnt++;
                break;


            default:
                if( RxPak->cmd_cnt < (RxPak->cmd_len) )
                    {
                    RxPak->command.cmdpak.cmd.data[RxPak->cmd_cnt-5] = MyComms->rxbuf[i];
                    RxPak->chk_sum = RxPak->chk_sum ^ MyComms->rxbuf[i];
                    RxPak->cmd_cnt++;
                    }
             }

        // Look for packet end / 2-April-13 added test for cmd_cmd > 0
        if( (RxPak->cmd_cnt > 0) && (RxPak->cmd_cnt == RxPak->cmd_len) )
            {
            if(MyComms->DebugRx)
                P3DebugPacket( RxPak );

            if( RxPak->chk_sum != 0 )
                {
                // 2-April-13, only nak if we are slave
                // checksum error
                if( MyComms->mode == kP3ModeSlave )
                    P3Command(MyComms, &Cmd_Nak_Chksum , RxPak->dev_id );
                }
            else
                P3DecodePacket( MyComms, &MyComms->RxPak );

            // clear timeout
            MyComms->rxto   = 0;
            RxPak->cmd_cnt  = 0;

            // slave is online - used in master mode only
            MyComms->online = 2;

            // See if there is a pending packet
            // used in master mode only
            if( MyComms->state == kP3StateReplyWaitTxPend )
                {
                P3SendPacket( MyComms, &MyComms->ExPak );
                MyComms->state = kP3StateReplyWait;
                }
            else
                {
                MyComms->state = kP3StateIdle;
                }
            }
        }
}

/*---------------------------------------------------------------------------*/
/*      Decode a received packet and take appropriate action                 */
/*---------------------------------------------------------------------------*/

void
P3DecodePacket( p3comms *MyComms, p3pak *packet )
{
    p3cmdfull   *cmd = &packet->command.cmdpak.cmd;
    int         ret = 0;

    // cmd not used now
    cmd = cmd;

    // Decoding in device specific code
    // If this returns positive then the command was handled
    if( MyComms->packet_decode != NULL )
        ret = MyComms->packet_decode( MyComms, packet );

    // Did user code handle the packet
    if(ret > 0)
        return;

    // Standard processing of common known commands
    switch( packet->masked_cmd1 )
        {
        case    CMD1_GROUP_SYSTEM_CMD:
            P3DecodeSysCtl( MyComms, packet );
            break;

        case    CMD1_GROUP_SYSTEM_REPLY:
            P3DecodeSysReply( MyComms, packet );
            break;

        default:
            // Nak - undefined command
            P3Command(MyComms, &Cmd_Nak_Und, packet->dev_id  );
            break;
        }
}

/*---------------------------------------------------------------------------*/
/*      Decode a received system control packet                              */
/*---------------------------------------------------------------------------*/

void
P3DecodeSysCtl( p3comms *MyComms, p3pak *packet )
{
    p3cmdfull   *cmd = &packet->command.cmdpak.cmd;

    //only action these if slave
    if( MyComms->mode == kP3ModeMaster )
        return;

    switch( cmd->cmd2 )
        {
        case    CMD2_SYSTEM_DEVICE_TYPE:
            // Dev type request
            Cmd_Dev_Type_Reply.data[0] = MyComms->deviceType[0];
            Cmd_Dev_Type_Reply.data[1] = MyComms->deviceType[1];
            P3Command(MyComms, &Cmd_Dev_Type_Reply, packet->dev_id  );
            break;

        case    CMD2_SYSTEM_MANUFACTURER:
            // manufacturer request
            Cmd_Manufacturer_Reply.length = strlen( (char *)MyComms->manufacturer ) + 1;
            strncpy( (char *)Cmd_Manufacturer_Reply.data, (char *)MyComms->manufacturer, Cmd_Manufacturer_Reply.length );
            P3Command(MyComms, &Cmd_Manufacturer_Reply, packet->dev_id  );
            break;

        case    CMD2_SYSTEM_PRODUCT_NAME:
            // product name request
            Cmd_ProductName_Reply.length = strlen( (char *)MyComms->product_name ) + 1;
            strncpy( (char *)Cmd_ProductName_Reply.data, (char *)MyComms->product_name, Cmd_ProductName_Reply.length );
            P3Command(MyComms, &Cmd_ProductName_Reply, packet->dev_id  );
            break;

        case    CMD2_SYSTEM_SERIAL_NUM:
            // serial number request
            Cmd_SerialNumber_Reply.length = strlen( (char *)MyComms->serial_number ) + 1;
            strncpy( (char *)Cmd_SerialNumber_Reply.data, (char *)MyComms->serial_number, Cmd_SerialNumber_Reply.length );
            P3Command(MyComms, &Cmd_SerialNumber_Reply, packet->dev_id  );
            break;

        case    CMD2_SYSTEM_FIRMWARE:
            // firmware version
            strncpy( (char *)Cmd_FirmwareVersion_Reply.data, (char *)MyComms->firmware_version, Cmd_FirmwareVersion_Reply.length );
            P3Command(MyComms, &Cmd_FirmwareVersion_Reply, packet->dev_id  );
            break;

        case    CMD2_SYSTEM_HARDWARE:
            // hardware version
            strncpy( (char *)Cmd_HardwareVersion_Reply.data,(char *) MyComms->hardware_version, Cmd_HardwareVersion_Reply.length );
            P3Command(MyComms, &Cmd_HardwareVersion_Reply, packet->dev_id  );
            break;

        default:
            // Nak - undefined command
            P3Command(MyComms, &Cmd_Nak_Und, packet->dev_id  );
            break;
        }
}

/*---------------------------------------------------------------------------*/
/*      Decode a received system reply packet                                */
/*---------------------------------------------------------------------------*/

void
P3DecodeSysReply( p3comms *MyComms, p3pak *packet )
{
    p3cmdfull   *cmd = &packet->command.cmdpak.cmd;
    int          len;

    // why are we here ??
    if( MyComms->mode == kP3ModeSlave )
        return;

    switch( cmd->cmd2 )
        {
        case    CMD2_SYSTEM_ACK:   // ACK
        case    CMD2_SYSTEM_NAK:   // NAK
            break;

        case    CMD2_SYSTEM_DEVICE_TYPE:
            // Dev type reply
            MyComms->deviceType[0] = packet->command.cmdpak.cmd.data[0];
            MyComms->deviceType[1] = packet->command.cmdpak.cmd.data[1];
            break;


        case   CMD2_SYSTEM_MANUFACTURER:
            // Manufacturer string
            len = packet->command.cmdpak.cmd.length;
            if( len > 31 )
                len = 31;
            strncpy( (char *)MyComms->manufacturer, (char *)packet->command.cmdpak.cmd.data, len );
            break;

       case   CMD2_SYSTEM_PRODUCT_NAME:
            // product name string
            len = packet->command.cmdpak.cmd.length;
            if( len > 31 )
                len = 31;
            strncpy( (char *)MyComms->product_name, (char *)packet->command.cmdpak.cmd.data, len );
            break;

       case   CMD2_SYSTEM_SERIAL_NUM:
            // serial number string
            len = packet->command.cmdpak.cmd.length;
            if( len > 31 )
                len = 31;
            strncpy( (char *)MyComms->serial_number, (char *)packet->command.cmdpak.cmd.data, len );
            break;

        case CMD2_SYSTEM_FIRMWARE:
            // firmware version
            strncpy( (char *)MyComms->firmware_version, (char *)packet->command.cmdpak.cmd.data, 5 );
            break;

        case CMD2_SYSTEM_HARDWARE:
            // hardware version
            strncpy( (char *)MyComms->hardware_version, (char *)packet->command.cmdpak.cmd.data, 5 );
            break;

        default:
            break;
        }
}

/*---------------------------------------------------------------------------*/
/*      Call this task often for communications                              */
/*---------------------------------------------------------------------------*/

int
P3CommsTask( p3comms *MyComms )
{
    int             rx_len;

    //Check for receive packet
    if( (rx_len = P3ReceiveData( MyComms )) > 0 )
        {
        P3ReceivePacket( MyComms );
        }
    else
        {
        if( MyComms->mode == kP3ModeMaster )
            {
            if( MyComms->state == kP3StateTimeout )
                {
                MyComms->state = kP3StateIdle;
                if( MyComms->online > 0 )
                   MyComms->online--;
                MyComms->tcount++;
                }
            }
        }

    return(P3_SUCCESS);
}

