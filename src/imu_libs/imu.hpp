#ifndef IMU_HPP
#define IMU_HPP

#include "Wire.h"
// #include "I2Cdev.h"
#include "I2Cdev/I2Cdev.h"
// #include "MPU6050/MPU6050_6Axis_MotionApps20.h"
#include "MPU6050/MPU6050_6Axis_MotionApps612.h"

#define TIMER_IMU 80        // The MPU is geting data every 4ms a multiple of 4 is better
#define I2C_BUFFER_SIZE 14

#include "MeConfig.h"
#ifdef ME_PORT_DEFINED
#include "MePort.h"
#endif // ME_PORT_DEFINED

// class Imu 
#ifndef ME_PORT_DEFINED
class Imu
#else // !ME_PORT_DEFINED
class Imu : public MePort
#endif // !ME_PORT_DEFINED
{
    //  private: 
    volatile uint8_t  _AD0;
    volatile uint8_t  _INT;
    
    uint8_t device_address;
    unsigned long timer;

    float  aSensitivity, aSensitivity_si; /* for 2g, check data sheet AFS_SEL = 0 Register 28 (0x1c) */
    float  gSensitivity, gSensitivity_si; /* for 500 deg/s, check data sheet */

    // class default I2C address is 0x68
    // specific I2C addresses may be passed as a parameter here
    // AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
    // AD0 high = 0x69
    MPU6050 mpu;

    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint8_t lastPacket[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 gyro;       // [x, y, z]            Gyro sensor measurements
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    // VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    void begin(uint8_t accel_config = MPU6050_ACCEL_FS_2, uint8_t gyro_config = MPU6050_GYRO_FS_500);

    float set_aSensitivity(uint8_t accel_config);
    float set_gSensitivity(uint8_t gyro_config);

  public:
    Imu(uint8_t port = 0x6);
    unsigned long read_timer(void);

    float get_aSensitivity(void);
    float get_aSensitivity_si(void);
    float get_gSensitivity(void);
    float get_gSensitivity_si(void);

    float get_roll(void);
    float get_pitch(void);
    float get_yaw(void);

    float get_angular_velocity_x(void);
    float get_angular_velocity_y(void);
    float get_angular_velocity_z(void);

    float get_linear_acceleration_x(void);
    float get_linear_acceleration_y(void);
    float get_linear_acceleration_z(void);

    void setup(void);
    bool loop(void);
};

#endif // IMU_HPP