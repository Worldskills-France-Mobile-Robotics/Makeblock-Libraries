#ifndef IMU_HPP
#define IMU_HPP

#include "Wire.h"
// #include "I2Cdev.h"
#include "I2Cdev/I2Cdev.h"
#include "MPU6050/MPU6050_Reduced.h"

#define TIMER_IMU 52        // The MPU is geting data every 4ms a multiple of 4 is better
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
    unsigned long timer;
    unsigned long	last_time;

    float  aSensitivity, aSensitivity_si; /* for 2g, check data sheet AFS_SEL = 0 Register 28 (0x1c) */
    float  gSensitivity, gSensitivity_si; /* for 500 deg/s, check data sheet */

    boolean first_loop;
    boolean set_gyro_angles;
    float acc_x, acc_y, acc_z, acc_total_vector;
    long acc_x_raw, acc_y_raw, acc_z_raw;
    long acc_x_cal, acc_y_cal, acc_z_cal;
    float temperature;
    float gyro_x, gyro_y, gyro_z;
    long gyro_x_raw, gyro_y_raw, gyro_z_raw;
    long gyro_x_cal, gyro_y_cal, gyro_z_cal;
    float angle_pitch, angle_yaw, angle_roll;
    float angle_pitch_output, angle_yaw_output, angle_roll_output;
    float angle_pitch_acc, angle_yaw_acc, angle_roll_acc;

    float gravity_calib[3];
    float gravity[3];       // [x, y, z]            gravity vector

    uint8_t i2cData[I2C_BUFFER_SIZE];
    uint8_t device_address;

    void begin(uint8_t accel_config = MPU6050_ACCEL_FS_2, uint8_t gyro_config = MPU6050_GYRO_FS_500);
    void calibrate(uint16_t calibration_iterations = 200);
    
    void read_mpu_6050_data(void);

    float set_aSensitivity(uint8_t accel_config);
    float set_gSensitivity(uint8_t gyro_config);

    void update(float tau = 0.5);

  public:
    Imu(uint8_t port = 0x6);
    unsigned long read_timer(void);

    float get_aSensitivity(void);
    float get_gSensitivity(void);

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
    void loop(void);
};

#endif // IMU_HPP