/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     p3demo.c                                                     */
/*    Author:     James Pearman                                                */
/*    Created:    13 August 2013                                               */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00  6 Sept 2013 - Initial release for ConVEX              */
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
/*    Communications Demo code for cortex                                      */
/*    This version has master and slave combined into one demo, master is on   */
/*    UART1, slave is on UART2                                                 */
/*                                                                             */
/*    This has no real world use, it's does stress the cortex and ROBOTC       */
/*    and allows the code to be run with one cortex, that's about it.          */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

#include "ch.h"         // needs for all ChibiOS programs
#include "hal.h"        // hardware abstraction layer header
#include "vex.h"        // vex library header
#include "robotc_glue.h"

#include "p3comms.h"    // p3comms header

#define MASTER_PORT     1
#define SLAVE_PORT      2

/*---------------------------------------------------------------------------*/
/*  define application specific commands                                     */
/*---------------------------------------------------------------------------*/

#define CMD2_CONTROL_SETMOTORS          0x10
#define CMD2_CONTROL_SET_MOTOR_BY_INDEX 0x11

#define CMD2_STATUS_GETMOTORS           0x10

//
static  p3cmd   Cmd_Set_Motors          = { CMD1_GROUP_CONTROL, CMD2_CONTROL_SETMOTORS,         10, {0} };
//static  p3cmd   Cmd_Set_Motor_ByIndex   = { CMD1_GROUP_CONTROL, CMD2_CONTROL_SET_MOTOR_BY_INDEX, 2, {0} };
static  p3cmd   Cmd_Motor_Status_Req    = { CMD1_GROUP_STATUS,  CMD2_STATUS_GETMOTORS,           0, {0} };
static  p3cmd   Cmd_Motor_Status        = { CMD1_GROUP_STATUS_REPLY, CMD2_STATUS_GETMOTORS,     10, {0} };

// System commands
static  p3cmd   Cmd_Ack                     = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_ACK,          0x01, {0x00} };
static  p3cmd   Cmd_Nak_Para_Err            = { CMD1_GROUP_SYSTEM_REPLY, CMD2_SYSTEM_NAK,          0x01, {0x08} };
static  p3cmd   Cmd_Dev_Type                = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_DEVICE_TYPE,  0, {0x00} };
static  p3cmd   Cmd_Manufacturer_Request    = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_MANUFACTURER, 0, {0x00} };
static  p3cmd   Cmd_ProductName_Request     = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_PRODUCT_NAME, 0, {0x00} };
static  p3cmd   Cmd_SerialNumber_Request    = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_SERIAL_NUM,   0, {0x00} };
static  p3cmd   Cmd_FirmwareRev_Request     = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_FIRMWARE,     0, {0x00} };
static  p3cmd   Cmd_HardwareRev_Request     = { CMD1_GROUP_SYSTEM_CMD, CMD2_SYSTEM_HARDWARE,     0, {0x00} };

// storage for the motor date we send to the slave
static  short   remote_motor[ kVexMotorNum ];

/*---------------------------------------------------------------------------*/
/*  Example control commands                                                 */
/*---------------------------------------------------------------------------*/

int
P3UserDecodeControl( p3comms *MyComms, p3pak *packet )
{
    p3cmdfull   *cmd = &packet->command.cmdpak.cmd;
    int          ret = 1;
    int          index;

    switch( cmd->cmd2 )
        {
        // Set all motors
        case   CMD2_CONTROL_SETMOTORS:
            // Check for valid data length
            if( cmd->length == 10 ) {
                // data is in range 0-254, shift to +/- 127
                vexMotorSet( kVexMotor_1, cmd->data[0] - 0x7F);
                vexMotorSet( kVexMotor_2, cmd->data[1] - 0x7F);
                vexMotorSet( kVexMotor_3, cmd->data[2] - 0x7F);
                vexMotorSet( kVexMotor_4, cmd->data[3] - 0x7F);
                vexMotorSet( kVexMotor_5, cmd->data[4] - 0x7F);
                vexMotorSet( kVexMotor_6, cmd->data[5] - 0x7F);
                vexMotorSet( kVexMotor_7, cmd->data[6] - 0x7F);
                vexMotorSet( kVexMotor_8, cmd->data[7] - 0x7F);
                vexMotorSet( kVexMotor_9, cmd->data[8] - 0x7F);
                vexMotorSet( kVexMotor_10,cmd->data[9] - 0x7F);

                // Send ACK
                P3Command(MyComms, &Cmd_Ack, packet->dev_id  );
                }
            else
                // Send NAK
                P3Command(MyComms, &Cmd_Nak_Para_Err, packet->dev_id  );
            break;

        // Set motor by index
        case   CMD2_CONTROL_SET_MOTOR_BY_INDEX:
            // Check for valid data length
            if( cmd->length == 2) {
                // first data byte is motor index in range 0 to 9
                index = cmd->data[0];
                // bounds check the index
                if( (index >= 0) && (index<=9) ) {
                    // data is in range 0-254, shift to +/- 127
                    vexMotorSet( index, cmd->data[1] - 0x7F);

                    // Send ACK
                    P3Command(MyComms, &Cmd_Ack, packet->dev_id  );
                    }
                else
                    P3Command(MyComms, &Cmd_Nak_Para_Err, packet->dev_id  );
                }
            else
                P3Command(MyComms, &Cmd_Nak_Para_Err, packet->dev_id  );
            break;

        default:
            ret = 0;
            break;
        }

    return(ret);
}

/*---------------------------------------------------------------------------*/
/*  Example status request and reply                                         */
/*---------------------------------------------------------------------------*/

int
P3UserDecodeStatus( p3comms *MyComms, p3pak *packet )
{
    p3cmdfull   *cmd = &packet->command.cmdpak.cmd;
    int          ret = 1;

    switch( cmd->cmd2 )
        {
        // Get all motors
        case    CMD2_STATUS_GETMOTORS:
            // shift +- 127 to 0-254 range
            Cmd_Motor_Status.data[0] = vexMotorGet( kVexMotor_1 ) + 0x7F;
            Cmd_Motor_Status.data[1] = vexMotorGet( kVexMotor_2 ) + 0x7F;
            Cmd_Motor_Status.data[2] = vexMotorGet( kVexMotor_3 ) + 0x7F;
            Cmd_Motor_Status.data[3] = vexMotorGet( kVexMotor_4 ) + 0x7F;
            Cmd_Motor_Status.data[4] = vexMotorGet( kVexMotor_5 ) + 0x7F;
            Cmd_Motor_Status.data[5] = vexMotorGet( kVexMotor_6 ) + 0x7F;
            Cmd_Motor_Status.data[6] = vexMotorGet( kVexMotor_7 ) + 0x7F;
            Cmd_Motor_Status.data[7] = vexMotorGet( kVexMotor_8 ) + 0x7F;
            Cmd_Motor_Status.data[8] = vexMotorGet( kVexMotor_9 ) + 0x7F;
            Cmd_Motor_Status.data[9] = vexMotorGet( kVexMotor_10) + 0x7F;
            // reply with motor status
            P3Command(MyComms, &Cmd_Motor_Status, packet->dev_id  );
            break;

        default:
            ret = 0;
            break;
        }

    return(ret);
}

/*---------------------------------------------------------------------------*/
/*  Example status reply decode                                              */
/*---------------------------------------------------------------------------*/

int
P3UserDecodeStatusReply( p3comms *MyComms, p3pak *packet )
{
    p3cmdfull   *cmd = &packet->command.cmdpak.cmd;
    int          ret = 1;

    (void)MyComms;

    switch( cmd->cmd2 )
        {
        // Get all motors
        case    CMD2_STATUS_GETMOTORS:
            // Check for valid data length
            if( cmd->length == 10 ) {
                // data is in range 0-254, shift to +/- 127
                remote_motor[kVexMotor_1]  = cmd->data[0] - 0x7F;
                remote_motor[kVexMotor_2]  = cmd->data[1] - 0x7F;
                remote_motor[kVexMotor_3]  = cmd->data[2] - 0x7F;
                remote_motor[kVexMotor_4]  = cmd->data[3] - 0x7F;
                remote_motor[kVexMotor_5]  = cmd->data[4] - 0x7F;
                remote_motor[kVexMotor_6]  = cmd->data[5] - 0x7F;
                remote_motor[kVexMotor_7]  = cmd->data[6] - 0x7F;
                remote_motor[kVexMotor_8]  = cmd->data[7] - 0x7F;
                remote_motor[kVexMotor_9]  = cmd->data[8] - 0x7F;
                remote_motor[kVexMotor_10] = cmd->data[9] - 0x7F;
                }
            break;

        default:
            ret = 0;
            break;
        }

    return(ret);
}

/*---------------------------------------------------------------------------*/
/*      Decode a received packet and take appropriate action                 */
/*---------------------------------------------------------------------------*/

int
P3UserDecodePacketMaster( p3comms *MyComms, p3pak *packet )
{
    int     ret = 1;

    // detect command group
    switch( packet->masked_cmd1 )
        {
        case    CMD1_GROUP_STATUS_REPLY:
            ret = P3UserDecodeStatusReply( MyComms, packet );
            break;

        default:
            ret = 0;
            break;
        }

    return(ret);
}

/*---------------------------------------------------------------------------*/
/*      Decode a received packet and take appropriate action                 */
/*---------------------------------------------------------------------------*/

int
P3UserDecodePacketSlave( p3comms *MyComms, p3pak *packet )
{
    int     ret = 1;
    static  long msgCount = 0;

    // detect command group
    switch( packet->masked_cmd1 )
        {
        case    CMD1_GROUP_CONTROL:
            ret = P3UserDecodeControl( MyComms, packet );
            break;

        case    CMD1_GROUP_STATUS:
            ret = P3UserDecodeStatus( MyComms, packet );
            break;

        default:
            ret = 0;
            break;
        }

    // Some debug - flash LED
    if(ret == 1)
        {
        msgCount++;
        vexDigitalPinSet( kVexDigital_1, (msgCount >> 5) & 1);
        }

    return(ret);
}

/*-----------------------------------------------------------------------------*/
/*  serial comms task for master                                               */
/*-----------------------------------------------------------------------------*/

p3comms  *MyCommsM;

task serialCommsTaskM(void *arg)
{
    (void) arg;

    // Must call this
    vexTaskRegister("comms master");

    // check for messages
    while( true )
        {
        // run communications
        if(MyCommsM != NULL)
            P3CommsTask( MyCommsM );

        // P3 comms task expects to be run every 2mS
        vexSleep(2);
        }
    return (msg_t)0;
}

/*-----------------------------------------------------------------------------*/
/*  Task that sends messages to the slave                                      */
/*-----------------------------------------------------------------------------*/

typedef enum {
    kStateIdle        =  0,
    kStateCheckOnline = 10,
    kStateCheckInit_1 = 20,
    kStateCheckInit_2,
    kStateCheckInit_3,
    kStateCheckInit_4,
    kStateCheckInit_5,
    kStateCheckInit_6,
    kStatePoll        = 30
    } commsState;

task serialMasterTask(void *arg)
{
    static  commsState  state;
            int     i;

    (void) arg;

    // Must call this
    vexTaskRegister("serial master");

    while(1)
        {
        if( state >= kStateCheckInit_1 )
            {
            if( MyCommsM->online == 0 )
                 state = kStateIdle;
            }

        switch(state)
            {
            case    kStateIdle:
                // Use device ID as the message to detect slave presence
                P3Command( MyCommsM, &Cmd_Dev_Type, CORTEX_DEVICE_ID  );
                state++;
                break;

            case    kStateCheckOnline:
                if( MyCommsM->online != 0 )
                    state = kStateCheckInit_1; // reply received so move on
                else
                    state = kStateIdle;        // Try again
                break;

            // Ask for all system related stuff just for fun.
            case    kStateCheckInit_1:
                P3Command( MyCommsM, &Cmd_Manufacturer_Request, CORTEX_DEVICE_ID  );
                state++;
                break;
            case    kStateCheckInit_2:
                P3Command( MyCommsM, &Cmd_ProductName_Request, CORTEX_DEVICE_ID  );
                state++;
                break;
            case    kStateCheckInit_3:
                P3Command( MyCommsM, &Cmd_SerialNumber_Request, CORTEX_DEVICE_ID  );
                state++;
                break;
            case    kStateCheckInit_4:
                P3Command( MyCommsM, &Cmd_FirmwareRev_Request, CORTEX_DEVICE_ID  );
                state++;
                break;
            case    kStateCheckInit_5:
                P3Command( MyCommsM, &Cmd_HardwareRev_Request, CORTEX_DEVICE_ID  );
                state++;
                break;

            case    kStateCheckInit_6:
                // Get current status of remote motors
                // we don't do much with this other than save when the reply arrives
                // if motors not under js control were running they will maintain
                // their current speed.
                P3Command( MyCommsM, &Cmd_Motor_Status_Req, CORTEX_DEVICE_ID  );
                state =     kStatePoll;
                break;

            case        kStatePoll:
                // for demo, move joystick data into motors 0 through 3
                remote_motor[0] = vexControllerGet( Ch1 );
                remote_motor[1] = vexControllerGet( Ch2 );
                remote_motor[2] = vexControllerGet( Ch3 );
                remote_motor[3] = vexControllerGet( Ch4 );

                // Send data for all motors
                for( i=0;i<10;i++ )
                    Cmd_Set_Motors.data[i] = remote_motor[i] + 0x7F;

                P3Command( MyCommsM, &Cmd_Set_Motors, CORTEX_DEVICE_ID  );
                break;

            default:
                state++;
                break;
            }

        vexSleep(10);
        }
    return (msg_t)0;
}

/*-----------------------------------------------------------------------------*/
/*  serial comms task for slave                                                */
/*-----------------------------------------------------------------------------*/

task serialCommsTaskS(void *arg)
{
    p3comms  *MyCommsS;

    (void)arg;

    // Must call this
    vexTaskRegister("comms slave");

    // Open comms in slave mode
    MyCommsS = P3Init( SLAVE_PORT, kP3ModeSlave, 0, 230400 );

    // Error - then quit
    if(MyCommsS == NULL)
        return (msg_t)0;


    // set device type - whatever you want
    MyCommsS->deviceType[0] = 0x12;
    MyCommsS->deviceType[1] = 0x34;

    // Set various system message details
    P3SetReplyDecoder( MyCommsS, P3UserDecodePacketSlave );
    P3SetManufacturerString( MyCommsS, "VEX");
    P3SetProductNameString( MyCommsS, "CORTEX");
    P3SetSerialNumberString( MyCommsS, "00001" );
    P3SetFirmwareVersion( MyCommsS, 1, 0, 0, 0);
    P3SetHardwareVersion( MyCommsS, 1, 0, 0 );

    // check for messages
    while( true )
        {
        // run communications
        if(MyCommsS != NULL)
            P3CommsTask( MyCommsS );

        // P3 comms task expects to be run every 2mS
        vexSleep(2);
        }
    return (msg_t)0;
}

/*-----------------------------------------------------------------------------*/
/*  Initialize slave                                                           */
/*-----------------------------------------------------------------------------*/

void
serialSlaveInit(void)
{
    StartTask(serialCommsTaskS, USER_THREAD_PRIORITY + 2);
}

/*-----------------------------------------------------------------------------*/
/*  Initialize master                                                          */
/*-----------------------------------------------------------------------------*/

void
serialMasterInit(void)
{
    // Open comms in master mode
    MyCommsM = P3Init( MASTER_PORT, kP3ModeMaster, 0, 230400 );

    // Start task if no error
    if(MyCommsM != NULL)
        {
        P3SetReplyDecoder( MyCommsM, P3UserDecodePacketMaster );

        StartTask(serialCommsTaskM, USER_THREAD_PRIORITY + 2 );
        StartTask(serialMasterTask, USER_THREAD_PRIORITY );
        }
}
