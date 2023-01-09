/*************************************************************************
* File Name          : Ruediger2.ino
* Author             : flochre
* Version            : 0.1
* Date               : 09/01/2023
* Description        : Firmware for Makeblock Electronic modules with Scratch.  
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 - 2016 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
* History:
* <Author>         <Time>         <Version>        <Descr>
* flochre          2023/01/09     0.1              First implementation with new IMU concept
**************************************************************************/
#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>

#include "imu_libs/imu.hpp"

#define DEBUG_INFO
//#define DEBUG_INFO1

MeMegaPiDCMotor dc;
MeStepperOnBoard steppers[4] = {MeStepperOnBoard(1),MeStepperOnBoard(2),MeStepperOnBoard(3),MeStepperOnBoard(4)};

MeUltrasonicSensor *us = NULL;     //PORT_7

Imu *my_imu = NULL;

MeEncoderOnBoard encoders[4];

typedef struct MeModule
{
  int16_t device;
  int16_t port;
  int16_t slot;
  int16_t pin;
  int16_t index;
  float values[3];
} MeModule;

union
{
  uint8_t byteVal[4];
  float floatVal;
  long longVal;
}val;

union
{
  uint8_t byteVal[8];
  double doubleVal;
}valDouble;

union
{
  uint8_t byteVal[2];
  int16_t shortVal;
}valShort;
MeModule modules[12];
#if defined(__AVR_ATmega32U4__) 
  int16_t analogs[12]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};
#endif
#if defined(__AVR_ATmega328P__) or defined(__AVR_ATmega168__)
  int16_t analogs[8]={A0,A1,A2,A3,A4,A5,A6,A7};
#endif
#if defined(__AVR_ATmega1280__)|| defined(__AVR_ATmega2560__)
  int16_t analogs[16]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15};
#endif

int16_t len = 52;

#define MOVE_STOP       0x00
#define MOVE_FORWARD    0x01
#define MOVE_BACKWARD   0x02

#define BLUETOOTH_MODE                       0x00
#define AUTOMATIC_OBSTACLE_AVOIDANCE_MODE    0x01
#define BALANCED_MODE                        0x02
#define IR_REMOTE_MODE                       0x03
#define LINE_FOLLOW_MODE                     0x04
#define MAX_MODE                             0x05

#define DATA_SERIAL                            0
#define DATA_SERIAL1                           1
#define DATA_SERIAL2                           2
#define DATA_SERIAL3                           3

uint8_t command_index = 0;
uint8_t megapi_mode = BLUETOOTH_MODE;
uint8_t index = 0;
uint8_t dataLen;
uint8_t modulesLen=0;
uint8_t irRead = 0;
uint8_t prevc=0;
uint8_t BluetoothSource = DATA_SERIAL;
uint8_t keyPressed = KEY_NULL;
uint8_t serialRead;
uint8_t buffer[52];
uint8_t bufferBt1[52];
uint8_t bufferBt2[52];
// double  lastTime = 0.0;
double  currentTime = 0.0;
double  CompAngleY, CompAngleX, GyroXangle;
double  LastCompAngleY, LastCompAngleX, LastGyroXangle;
double  last_turn_setpoint_filter = 0.0;
double  last_speed_setpoint_filter = 0.0;
double  last_speed_error_filter = 0.0;
double  speed_Integral_average = 0.0;
double  angle_speed = 0.0;

float angleServo = 90.0;
float dt;

long lasttime_angle = 0;
long lasttime_speed = 0;
long update_sensor = 0;
long blink_time = 0;
long last_Pulse_pos_encoder1 = 0;
long last_Pulse_pos_encoder2 = 0;

boolean isStart = false;
boolean isAvailable = false;
boolean leftflag;
boolean rightflag;
boolean start_flag = false;
boolean move_flag = false;
boolean blink_flag = false;

String mVersion = "0.1";
//////////////////////////////////////////////////////////////////////////////////////
float RELAX_ANGLE = -1;                    //Natural balance angle,should be adjustment according to your own car

#define VERSION                0
#define ULTRASONIC_SENSOR      1
#define TEMPERATURE_SENSOR     2
#define LIGHT_SENSOR           3
#define POTENTIONMETER         4
#define JOYSTICK               5
#define GYRO                   6
#define SOUND_SENSOR           7
#define RGBLED                 8
#define SEVSEG                 9
#define MOTOR                  10
#define SERVO                  11
#define ENCODER                12
#define IR                     13
#define IMU                    14
#define PIRMOTION              15
#define INFRARED               16
#define LINEFOLLOWER           17
#define SHUTTER                20
#define LIMITSWITCH            21
#define BUTTON                 22
#define HUMITURE               23
#define FLAMESENSOR            24
#define GASSENSOR              25
#define COMPASS                26
#define TEMPERATURE_SENSOR_1   27
#define DIGITAL                30
#define ANALOG                 31
#define PWM                    32
#define SERVO_PIN              33
#define TONE                   34
#define BUTTON_INNER           35
#define ULTRASONIC_ARDUINO     36
#define PULSEIN                37
#define STEPPER                40
#define LEDMATRIX              41
#define TIMER                  50
#define TOUCH_SENSOR           51
#define JOYSTICK_MOVE          52
#define COMMON_COMMONCMD       60
  //Secondary command
  #define SET_STARTER_MODE     0x10
  #define SET_AURIGA_MODE      0x11
  #define SET_MEGAPI_MODE      0x12
  #define GET_BATTERY_POWER    0x70
  #define GET_AURIGA_MODE      0x71
  #define GET_MEGAPI_MODE      0x72
#define ENCODER_BOARD 61
  //Read type
  #define ENCODER_BOARD_POS    0x01
  #define ENCODER_BOARD_SPEED  0x02

#define ENCODER_PID_MOTION     62
  //Secondary command
  #define ENCODER_BOARD_POS_MOTION_MOVE    0x01
  #define ENCODER_BOARD_SPEED_MOTION       0x02
  #define ENCODER_BOARD_PWM_MOTION         0x03
  #define ENCODER_BOARD_SET_CUR_POS_ZERO   0x04
  #define ENCODER_BOARD_CAR_POS_MOTION     0x05
  #define ENCODER_BOARD_POS_MOTION_MOVETO  0x06

#define TWO_ENCODERS_POS_SPEED 63

#define STEPPER_NEW            76
  //Secondary command
  #define STEPPER_POS_MOTION_MOVE          0x01
  #define STEPPER_SPEED_MOTION             0x02
  #define STEPPER_SET_CUR_POS_ZERO         0x04
  #define STEPPER_POS_MOTION_MOVETO        0x06

#define COLORSENSOR            67
  //Secondary command
  #define GETRGB                           0x01
  #define GETBOOL                          0x02
  #define GETCOLOR                         0x03

#define GET 1
#define RUN 2
#define RESET 4
#define START 5

typedef struct
{
  double P, I, D;
  double Setpoint, Output, Integral,differential, last_error;
} PID;

PID  PID_angle, PID_speed, PID_turn;


/************************************************************/
/************************************************************/
// Defining an LED pin to show the status of IMU data read
#define LED_PIN 13
#define TIMER_LED 250
unsigned long timer_led = 0;
bool blinkState = false;

/************************************************************/
/************************************************************/


/**
 * \par Function
 *    encoder_move_finish_callback
 * \par Description
 *    This function called when encoder motor move finish.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void encoder_move_finish_callback(int slot,int extId)
{
  writeHead();
  writeSerial(extId);
  sendByte(slot);
  writeEnd();
}

/**
 * \par Function
 *    stepper_move_finish_callback
 * \par Description
 *    This function called when stepper motor move finish.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void stepper_move_finish_callback(int slot,int extId)
{
  writeHead();
  writeSerial(extId);
  sendByte(slot);
  writeEnd();
}
/**
 * \par Function
 *    isr_process_encoder1
 * \par Description
 *    This function use to process the interrupt of encoder1 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder1 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder1(void)
{
  if(digitalRead(encoders[0].getPortB()) == 0)
  {
    encoders[0].pulsePosMinus();
  }
  else
  {
    encoders[0].pulsePosPlus();;
  }
}

/**
 * \par Function
 *    isr_process_encoder2
 * \par Description
 *    This function use to process the interrupt of encoder2 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder2 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder2(void)
{
  if(digitalRead(encoders[1].getPortB()) == 0)
  {
    encoders[1].pulsePosMinus();
  }
  else
  {
    encoders[1].pulsePosPlus();
  }
}

/**
 * \par Function
 *    isr_process_encoder3
 * \par Description
 *    This function use to process the interrupt of encoder3 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder3 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder3(void)
{
  if(digitalRead(encoders[2].getPortB()) == 0)
  {
    encoders[2].pulsePosMinus();
  }
  else
  {
    encoders[2].pulsePosPlus();
  }
}

/**
 * \par Function
 *    isr_process_encoder4
 * \par Description
 *    This function use to process the interrupt of encoder4 drvicer on board,
 *    used to calculate the number of pulses.
 * \param[in]
 *    None
 * \par Output
 *    The number of pulses on encoder4 driver
 * \return
 *    None
 * \par Others
 *    None
 */
void isr_process_encoder4(void)
{
  if(digitalRead(encoders[3].getPortB()) == 0)
  {
    encoders[3].pulsePosMinus();
  }
  else
  {
    encoders[3].pulsePosPlus();
  }
}

/**
 * \par Function
 *    readBuffer
 * \par Description
 *    This function use to read the serial data from its buffer..
 * \param[in]
 *    index - The first address in the array
 * \par Output
 *    None
 * \return
 *    The data need to be read.
 * \par Others
 *    None
 */
uint8_t readBuffer(int16_t index)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    return buffer[index];
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    return bufferBt1[index];
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    return bufferBt2[index];
  }
}

/**
 * \par Function
 *    writeBuffer
 * \par Description
 *    This function use to write the serial data to its buffer..
 * \param[in]
 *    index - The data's first address in the array
  * \param[in]
 *    c - The data need to be write.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeBuffer(int16_t index,uint8_t c)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    buffer[index]=c;
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    bufferBt1[index]=c;
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    bufferBt2[index]=c;
  }
}

/**
 * \par Function
 *    writeHead
 * \par Description
 *    This function use to write the head of transmission frame.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeHead(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
}

/**
 * \par Function
 *    writeEnd
 * \par Description
 *    This function use to write the terminator of transmission frame.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeEnd(void)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    Serial.println();
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    Serial2.println();
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    Serial3.println();
  }
}

/**
 * \par Function
 *    writeSerial
 * \par Description
 *    This function use to write the data to serial.
 * \param[in]
 *    c - The data need to be write.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void writeSerial(uint8_t c)
{
  if(BluetoothSource == DATA_SERIAL)
  {
    Serial.write(c);
  }
  else if(BluetoothSource == DATA_SERIAL2)
  {
    Serial2.write(c);
  }
  else if(BluetoothSource == DATA_SERIAL3)
  {
    Serial3.write(c);
  }
}

/**
 * \par Function
 *    readSerial
 * \par Description
 *    This function use to read the data from serial, and fill the data
 *    to its buffer.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void readSerial(void)
{
  isAvailable = false;
  if(Serial.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL;
    serialRead = Serial.read();
  }
  else if(Serial2.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL2;
    serialRead = Serial2.read();
  }
  else if(Serial3.available() > 0)
  {
    isAvailable = true;
    BluetoothSource = DATA_SERIAL3;
    serialRead = Serial3.read();
  }
}

/**
 * \par Function
 *    parseData
 * \par Description
 *    This function use to process the data from the serial port,
 *    call the different treatment according to its action.
 *    ff 55 len idx action device port  slot  data a
 *    0  1  2   3   4      5      6     7     8
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void parseData(void)
{
  isStart = false;
  uint8_t idx = readBuffer(3);
  uint8_t action = readBuffer(4);
  uint8_t device = readBuffer(5);
  command_index = (uint8_t)idx;
  switch(action)
  {
    case GET:
      {
        readSensor(device);
        writeEnd();
      }
      break;
    case RUN:
      {
        runModule(device);
        callOK();
      }
      break;
    case RESET:
      {
        //reset
        /* reset On-Board encoder driver */
        for(int i=0;i<4;i++)
        {
          encoders[i].setPulsePos(0);
          encoders[i].moveTo(0,10);
          encoders[i].setMotorPwm(0);
          encoders[i].setMotionMode(DIRECT_MODE);
          steppers[i].setCurrentPosition(0);
          steppers[i].moveTo(0);
          steppers[i].disableOutputs();
        }

        /* reset dc motor on driver port */
        dc.reset(PORT1A);
        dc.run(0);
        dc.reset(PORT1B);
        dc.run(0);
        dc.reset(PORT2A);
        dc.run(0);
        dc.reset(PORT2B);
        dc.run(0);
        dc.reset(PORT3A);
        dc.run(0);
        dc.reset(PORT3B);
        dc.run(0);
        dc.reset(PORT4A);
        dc.run(0);
        dc.reset(PORT4B);
        dc.run(0);

        /* reset stepper motor driver */
        
        callOK();
      }
      break;
     case START:
      {
        //start
        callOK();
      }
      break;
  }
}

/**
 * \par Function
 *    callOK
 * \par Description
 *    Response for executable commands.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void callOK(void)
{
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}

/**
 * \par Function
 *    sendByte
 * \par Description
 *    Send byte data
 * \param[in]
 *    c - the byte data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendByte(uint8_t c)
{
  writeSerial(1);
  writeSerial(c);
}

/**
 * \par Function
 *    sendString
 * \par Description
 *    Send string data
 * \param[in]
 *    s - the string data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendString(String s)
{
  int16_t l = s.length();
  writeSerial(4);
  writeSerial(l);
  for(int16_t i=0;i<l;i++)
  {
    writeSerial(s.charAt(i));
  }
}

/**
 * \par Function
 *    sendFloat
 * \par Description
 *    Sned float data
 * \param[in]
 *    value - the float data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendFloat(float value)
{ 
  writeSerial(2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

/**
 * \par Function
 *    sendLong
 * \par Description
 *    Sned long data
 * \param[in]
 *    value - the long data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendLong(long value)
{ 
  writeSerial(6);
  val.longVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}

/**
 * \par Function
 *    sendShort
 * \par Description
 *    Sned short data
 * \param[in]
 *    value - the short data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendShort(int16_t value)
{
  writeSerial(3);
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
}

/**
 * \par Function
 *    sendDouble
 * \par Description
 *    Sned double data, same as float data on arduino.
 * \param[in]
 *    value - the double data need be sent.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void sendDouble(double value)
{
  writeSerial(5);
  valDouble.doubleVal = value;
  writeSerial(valDouble.byteVal[0]);
  writeSerial(valDouble.byteVal[1]);
  writeSerial(valDouble.byteVal[2]);
  writeSerial(valDouble.byteVal[3]);
}

/**
 * \par Function
 *    readShort
 * \par Description
 *    read the short data.
 * \param[in]
 *    idx - The data's first address in the array.
 * \par Output
 *    None
 * \return
 *    the short data.
 * \par Others
 *    None
 */
int16_t readShort(int16_t idx)
{
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx+1);
  return valShort.shortVal; 
}

/**
 * \par Function
 *    readFloat
 * \par Description
 *    read the float data.
 * \param[in]
 *    idx - The data's first address in the array.
 * \par Output
 *    None
 * \return
 *    the float data.
 * \par Others
 *    None
 */
float readFloat(int16_t idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.floatVal;
}

/**
 * \par Function
 *    readLong
 * \par Description
 *    read the long data.
 * \param[in]
 *    idx - The data's first address in the array.
 * \par Output
 *    None
 * \return
 *    the long data.
 * \par Others
 *    None
 */
long readLong(int16_t idx)
{
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx+1);
  val.byteVal[2] = readBuffer(idx+2);
  val.byteVal[3] = readBuffer(idx+3);
  return val.longVal;
}

char _receiveStr[20] = {};
uint8_t _receiveUint8[16] = {};

/**
 * \par Function
 *    readString
 * \par Description
 *    read the string data.
 * \param[in]
 *    idx - The string's first address in the array.
 * \param[in]
 *    len - The length of the string data.
 * \par Output
 *    None
 * \return
 *    the address of string data.
 * \par Others
 *    None
 */
char* readString(int16_t idx,int16_t len)
{
  for(int16_t i=0;i<len;i++)
  {
    _receiveStr[i]=readBuffer(idx+i);
  }
  _receiveStr[len] = '\0';
  return _receiveStr;
}

/**
 * \par Function
 *    readUint8
 * \par Description
 *    read the uint8 data.
 * \param[in]
 *    idx - The Uint8 data's first address in the array.
 * \param[in]
 *    len - The length of the uint8 data.
 * \par Output
 *    None
 * \return
 *    the address of uint8 data.
 * \par Others
 *    None
 */
uint8_t* readUint8(int16_t idx,int16_t len)
{
  for(int16_t i=0;i<len;i++)
  {
    if(i > 15)
    {
      break;
    }
    _receiveUint8[i] = readBuffer(idx+i);
  }
  return _receiveUint8;
}

/**
 * \par Function
 *    initStepper
 * \par Description
 *    Initialize acceleration, subdivision, and speed for stepper motor.
 * \param[in]
 *    index - The index of stepper.
 * \param[in]
 *    maxSpeed - The max speed of stepper.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void initStepper(uint8_t index,int16_t maxSpeed)
{
  // steppers[index].setpin(index+1);

  steppers[index].setMaxSpeed(maxSpeed);
  steppers[index].setAcceleration(20000);
  steppers[index].setMicroStep(16);
  steppers[index].setSpeed(maxSpeed);
  steppers[index].enableOutputs();
}
/**
 * \par Function
 *    runModule
 * \par Description
 *    Processing execute commands.
 * \param[in]
 *    device - The definition of all execute commands.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void runModule(uint8_t device)
{
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
  uint8_t port = readBuffer(6);
  uint8_t pin = port;
  switch(device)
  {
    case MOTOR:
      {
        int16_t speed = readShort(7);
        dc.reset(port);
        dc.run(speed);
      }
      break;
    case ENCODER_BOARD:
      if(port == 0)
      {
        uint8_t slot = readBuffer(7);
        int16_t speed_value = readShort(8);
        speed_value = -speed_value;
        encoders[slot-1].setTarPWM(speed_value);
      }
      break;
    case ENCODER_PID_MOTION:
      {
        uint8_t subcmd = port;
        uint8_t extID = readBuffer(3);
        uint8_t slot_num = readBuffer(7);
        if(ENCODER_BOARD_POS_MOTION_MOVE == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          speed_temp = abs(speed_temp);
          encoders[slot_num-1].move(pos_temp,(float)speed_temp,extID,encoder_move_finish_callback);
        }
        if(ENCODER_BOARD_POS_MOTION_MOVETO == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          speed_temp = abs(speed_temp);
          encoders[slot_num-1].moveTo(pos_temp,(float)speed_temp,extID,encoder_move_finish_callback);
        }
        else if(ENCODER_BOARD_SPEED_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);  
          encoders[slot_num-1].runSpeed((float)speed_temp);
        }
        else if(ENCODER_BOARD_PWM_MOTION == subcmd)
        {
          int16_t speed_temp = readShort(8);  
          encoders[slot_num-1].setTarPWM(speed_temp);     
        }
        else if(ENCODER_BOARD_SET_CUR_POS_ZERO == subcmd)
        {
          encoders[slot_num-1].setPulsePos(0);     
        }
        else if(ENCODER_BOARD_CAR_POS_MOTION == subcmd)
        {
          long pos_temp = readLong(8);
          int16_t speed_temp = readShort(12);
          if(slot_num == 1)
          {
            encoders[0].move(pos_temp,(float)speed_temp);
            encoders[1].move(-pos_temp,(float)speed_temp);
          }
          else if(slot_num == 2)
          {
            encoders[0].move(-pos_temp,(float)speed_temp);
            encoders[1].move(pos_temp,(float)speed_temp);
          }
          else if(slot_num == 3)
          {
            encoders[0].move(pos_temp,(float)speed_temp);
            encoders[1].move(pos_temp,(float)speed_temp);
          }
          else if(slot_num == 4)
          {
            encoders[0].move(-pos_temp,(float)speed_temp);
            encoders[1].move(-pos_temp,(float)speed_temp);
          }
        }
      }
      break;
  }
}


/**
 * \par Function
 *    readSensor
 * \par Description
 *    This function is used to process query command.
 * \param[in]
 *    device - The definition of all query commands.
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void readSensor(uint8_t device)
{
    /**************************************************
         ff 55 len idx action device port slot data a
        0  1  2   3   4      5      6    7    8
    ***************************************************/
    float value=0.0;
    uint8_t port,slot,pin;
    port = readBuffer(6);
    pin = port;
    writeHead();
    writeSerial(command_index);
    switch(device)
    {
        case ULTRASONIC_SENSOR:
        {
            if(us == NULL)
            {
                us = new MeUltrasonicSensor(port);
            }
            else if(us->getPort() != port)
            {
                delete us;
                us = new MeUltrasonicSensor(port);
            }
            value = (float)us->distanceCm();
            sendFloat(value);
        }
        break;
        case IMU:
        {
            if(NULL != my_imu){
                sendDouble(my_imu->get_roll());
                sendDouble(my_imu->get_pitch());
                sendDouble(my_imu->get_yaw());

                sendDouble(my_imu->get_angular_velocity_x());
                sendDouble(my_imu->get_angular_velocity_y());
                sendDouble(my_imu->get_angular_velocity_z());

                sendDouble(my_imu->get_linear_acceleration_x());
                sendDouble(my_imu->get_linear_acceleration_y());
                sendDouble(my_imu->get_linear_acceleration_z());
            }

        }
        break;
        case GYRO:
        {
            if(NULL != my_imu && port == my_imu->getPort()){
                delete my_imu;
                my_imu = new Imu(port);
                my_imu->setup();
            }

            if(NULL == my_imu){
                my_imu = new Imu(port);
                my_imu->setup();
            }

            if(NULL != my_imu && port == (my_imu->getPort() + 0x10) ){
                delete my_imu;
            }
        }
        break;
        
        case VERSION:
        {
            sendString(mVersion);
        }
        break;
        case DIGITAL:
        {
            pinMode(pin,INPUT);
            sendFloat(digitalRead(pin));
        }
        break;
        case ANALOG:
        {
            pin = analogs[pin];
            pinMode(pin,INPUT);
            sendFloat(analogRead(pin));
        }
        break;
        case PULSEIN:
        {
            int16_t pw = readShort(7);
            pinMode(pin, INPUT);
            sendLong(pulseIn(pin,HIGH,pw));
        }
        break;
        case ULTRASONIC_ARDUINO:
        {
            uint8_t trig = readBuffer(6);
            uint8_t echo = readBuffer(7);
            long pw_data;
            float dis_data;
            pinMode(trig,OUTPUT);
            digitalWrite(trig,LOW);
            delayMicroseconds(2);
            digitalWrite(trig,HIGH);
            delayMicroseconds(10);
            digitalWrite(trig,LOW);
            pinMode(echo, INPUT);
            pw_data = pulseIn(echo,HIGH,30000);
            dis_data = pw_data/58.0;
            delay(5);
            writeHead();
            writeSerial(command_index);
            sendFloat(pw_data);
        }
        break;
        case TIMER:
        {
            sendFloat((float)currentTime);
        }
        break;
        case ENCODER_BOARD:
        {
            if(port == 0)
            {
            slot = readBuffer(7);
            uint8_t read_type = readBuffer(8);
            if(read_type == ENCODER_BOARD_POS)
            {
                sendLong(encoders[slot-1].getCurPos());
            }
            else if(read_type == ENCODER_BOARD_SPEED)
            {
                sendFloat(encoders[slot-1].getCurrentSpeed());
            }
            }
        }
        break;
        case TWO_ENCODERS_POS_SPEED:
        {
            if(port == 0)
            {
            int16_t slot_1 = readBuffer(7);
            int16_t slot_2 = readBuffer(8);
            sendLong(encoders[slot_1-1].getCurPos());
            sendFloat(encoders[slot_1-1].getCurrentSpeed());
            sendLong(encoders[slot_2-1].getCurPos());
            sendFloat(encoders[slot_2-1].getCurrentSpeed());
            }
        }
        break;
        case COMMON_COMMONCMD:
        {
            uint8_t subcmd = port;
            if(GET_MEGAPI_MODE == subcmd)
            {
            sendByte(megapi_mode);
            }
        }
        break;
        default:
        {
            sendFloat(0);
        }
        break;
    }//switch
}

uint8_t buf[64];
uint8_t bufindex;

/**
 * \par Function
 *    read_serial
 * \par Description
 *    The function used to process serial data.
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    Is there a valid command 
 * \par Others
 *    None
 */
boolean read_serial(void)
{
  boolean result = false;
  readSerial();
  if(isAvailable)
  {
    uint8_t c = serialRead & 0xff;
    result = true;
    if((c == 0x55) && (isStart == false))
    {
      if(prevc == 0xff)
      {
        index=1;
        isStart = true;
      }
    }
    else
    {
      prevc = c;
      if(isStart)
      {
        if(index == 2)
        {
          dataLen = c; 
        }
        else if(index > 2)
        {
          dataLen--;
        }
        writeBuffer(index,c);
      }
    }
    index++;
    if(index > 51)
    {
      index=0; 
      isStart=false;
    }
    if(isStart && (dataLen == 0) && (index > 3))
    { 
      isStart = false;
      parseData(); 
      index=0;
    }
    return result;
  }
}
void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    while(!Serial){}
    while(!Serial2){}
    while(!Serial3){}
    delay(5);

    for(int i=0;i<4;i++){
        encoders[i].reset(i+1);
    }

    attachInterrupt(encoders[0].getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(encoders[1].getIntNum(), isr_process_encoder2, RISING);
    attachInterrupt(encoders[2].getIntNum(), isr_process_encoder3, RISING);
    attachInterrupt(encoders[3].getIntNum(), isr_process_encoder4, RISING);
    
    delay(5);
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // Set PWM over 20kHz
    // Timer 1 : PWM = PWM, Phase Correct 8 bits -> PWM = 1 -> WGM(3:0) = 0b0001
    // Timer 1 : Prescaler = 1, CS(2:0) = 0b001
    // Timer 1 Changes SLOT1 (Motor1)
    // TCCR1A  COM1A1  COM1A0  COM1B1  COM1B0  COM1C1  COM1C0  WGM11   WGM10
    // TCCR1B  ICNC1   ICES1   -       WGM13   WGM12   CS12    CS11    CS10
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS10);
    TCCR1C = 0;

    // Timer 2 : PWM = PWM, Phase Correct -> PWM = 1 or 5 -> WGM(2:0) = 0b001
    // Timer 2 : Prescaler = 1, CS(2:0) = 0b001
    // Timer 2 Changes SLOT3 (Motor3)
    // TCCR2A  COM2A1  COM2A0  COM2B1  COM2B0  -       -       WGM21   WGM20
    // TCCR2B  FOC2A   FOC2B   -       -       WGM22   CS22    CS21    CS20
    TCCR2A = _BV(WGM20);
    TCCR2B = _BV(CS20);

    // Timer 3 : PWM = PWM, Phase Correct 8 bits -> PWM = 1 -> WGM(3:0) = 0b0001
    // Timer 3 : Prescaler = 1, CS(2:0) = 0b001
    // Timer 3 Changes SLOT4 (Motor4)
    // TCCR3A  COM3A1  COM3A0  COM3B1  COM3B0  COM3C1  COM3C0  WGM31   WGM30
    // TCCR3B  ICNC3   ICES3   -       WGM33   WGM32   CS32    CS31    CS30
    TCCR3A = _BV(WGM30);
    TCCR3B = _BV(CS30);
    TCCR3C = 0;

    // Timer 4 : PWM = PWM, Phase Correct 8 bits -> PWM = 1 -> WGM(3:0) = 0b0001
    // Timer 4 : Prescaler = 1, CS(2:0) = 0b001
    // Timer 4 Changes SLOT2 (Motor2)
    // TCCR4A  COM4A1  COM4A0  COM4B1  COM4B0  COM4C1  COM4C0  WGM41   WGM40
    // TCCR4B  ICNC4   ICES4   -       WGM43   WGM42   CS42    CS41    CS40
    TCCR4A = _BV(WGM40);
    TCCR4B = _BV(CS40);
    TCCR4C = 0;

    for(int i=0;i<4;i++)
    {
        encoders[i].setPulse(8);
        encoders[i].setRatio(46.67);
        encoders[i].setPosPid(1.8,0,1.2);
        // encoders[i].setSpeedPid(0.8, 0.5, 0.001);
        encoders[i].setSpeedPid(0.4, 0.5, 0.00002);
        encoders[i].setMotionMode(PID_MODE);
    }

    leftflag=false;
    rightflag=false;
    PID_angle.Setpoint = RELAX_ANGLE;
    PID_angle.P = 20;          // 20;
    PID_angle.I = 1;           // 1;
    PID_angle.D = 0.2;         // 0.2;
    PID_speed.P = 0.06;        // 0.06
    PID_speed.I = 0.005;       // 0.005

    Serial.print("Version: ");
    Serial.println(mVersion);
    update_sensor = lasttime_speed = lasttime_angle = millis();
    blink_time = millis();
    BluetoothSource = DATA_SERIAL;
}

/**
 * \par Function
 *    loop
 * \par Description
 *    main function for arduino
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    None
 * \par Others
 *    None
 */
void loop(){

    for(int i=0;i<4;i++){
        steppers[i].update();
        encoders[i].loop();
    }

    if (NULL != my_imu && millis() >= my_imu->read_timer() + TIMER_IMU){
        my_imu->loop();
        // the readSensor fonction will send the data over serial
        readSensor(IMU);
    }

    readSerial();
    while(isAvailable)
    {
        unsigned char c = serialRead & 0xff;
        if((c == 0x55) && (isStart == false)){
            if(prevc == 0xff)
            {
                index=1;
                isStart = true;
            }
        }
        else{
            prevc = c;
            if(isStart)
            {
                if(index == 2){
                    dataLen = c; 
                }
                else if(index > 2){
                    dataLen--;
                }
                writeBuffer(index,c);
            }
        }

        index++;
        if(index > 51){
            index=0; 
            isStart=false;
        }
        if(isStart && (dataLen == 0) && (index > 3)){ 
            isStart = false;
            parseData(); 
            index=0;
        }
        readSerial();
    }

    if (millis() >= timer_led + TIMER_LED){
        timer_led = millis();
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
  
}
