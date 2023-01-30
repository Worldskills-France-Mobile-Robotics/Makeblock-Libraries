#include "imu.hpp"

Imu::Imu(uint8_t port) : MePort(port){
    timer = millis();
    device_address = MPU6050_DEFAULT_ADDRESS;

    first_loop = true;
    set_gyro_angles = false;

    #ifdef ME_PORT_DEFINED
    _AD0 = s1;
    _INT = s2;
    #endif // ME_PORT_DEFINED

    // Set _AD0 to the ground
    // https://www.i2cdevlib.com/forums/topic/414-freezing-problem/
    MePort::dWrite1(LOW);
}

void Imu::begin(uint8_t accel_config, uint8_t gyro_config) {
    aSensitivity = set_aSensitivity(accel_config);
    gSensitivity = set_gSensitivity(gyro_config);

    first_loop = true;
    set_gyro_angles = false;
    angle_pitch = 0, angle_yaw = 0, angle_roll = 0;
    angle_pitch_acc = 0, angle_yaw_acc = 0, angle_roll_acc = 0;

    Wire.begin();

    delay(200);
    //close the sleep mode
    I2Cdev::writeByte(device_address, MPU6050_RA_PWR_MGMT_1, 0x00);

    delay(100);
    //configurate the digital low pass filter
    I2Cdev::writeByte(device_address, MPU6050_RA_CONFIG, 0x01);

    //set the accel scale
    I2Cdev::writeBits(device_address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, accel_config);
    //set the gyro scale
    I2Cdev::writeBits(device_address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, gyro_config);

    delay(100);
    calibrate(200);

}

void Imu::calibrate(uint16_t calibration_iterations){
    acc_x_cal = 0, acc_y_cal = 0, acc_z_cal = 0;
    gyro_x_cal = 0, gyro_y_cal = 0, gyro_z_cal = 0;

    for (size_t i = 0; i < calibration_iterations; i++) {
        read_mpu_6050_data();
        acc_x_cal += acc_x_raw;
        acc_y_cal += acc_y_raw;
        acc_z_cal += acc_z_raw;

        gyro_x_cal += gyro_x_raw;
        gyro_y_cal += gyro_y_raw;
        gyro_z_cal += gyro_z_raw;
        delay(4);
    }
  
    acc_x_cal /= calibration_iterations;
    acc_y_cal /= calibration_iterations;
    acc_z_cal /= calibration_iterations;
    
    gyro_x_cal /= calibration_iterations;
    gyro_y_cal /= calibration_iterations;
    gyro_z_cal /= calibration_iterations;

}

void Imu::read_mpu_6050_data(void){
    /* read imu data */
    if(!I2Cdev::readBytes(device_address, MPU6050_RA_ACCEL_XOUT_H, I2C_BUFFER_SIZE, i2cData)) {
        return;
    }

    acc_x_raw   = ( (i2cData[0] << 8) | i2cData[1] );
    acc_y_raw   = ( (i2cData[2] << 8) | i2cData[3] );
    acc_z_raw   = ( (i2cData[4] << 8) | i2cData[5] );

    temperature = ( (i2cData[6] << 8) | i2cData[7] );

    gyro_x_raw  = ( (i2cData[8] << 8)  | i2cData[9]  );
    gyro_y_raw  = ( (i2cData[10] << 8) | i2cData[11] );
    gyro_z_raw  = ( (i2cData[12] << 8) | i2cData[13] );
}

float Imu::get_aSensitivity(void){
    return aSensitivity;
}

float Imu::set_aSensitivity(uint8_t accel_config){
    switch (accel_config) {
        case MPU6050_ACCEL_FS_2:
            aSensitivity = 16384.0;
            break;
        
        case MPU6050_ACCEL_FS_4:
            aSensitivity = 8192.0;
            break;

        case MPU6050_ACCEL_FS_8:
            aSensitivity = 4096.0;
            break;

        case MPU6050_ACCEL_FS_16:
            aSensitivity = 2048.0;
            break;

        default:
            aSensitivity = 16384.0;
            break;
    }

    aSensitivity_si = aSensitivity / 9.8; // LSB/g to LSB/ms-2 

    return aSensitivity;
}

float Imu::get_gSensitivity(void){
    return gSensitivity;
}

float Imu::set_gSensitivity(uint8_t gyro_config){
    switch (gyro_config)
    {
        case MPU6050_GYRO_FS_250:
            gSensitivity = 131.0;
            break;
        
        case MPU6050_GYRO_FS_500:
            gSensitivity = 65.5;
            break;

        case MPU6050_GYRO_FS_1000:
            gSensitivity = 32.8;
            break;

        case MPU6050_GYRO_FS_2000:
            gSensitivity = 16.4;
            break;

        default:
            gSensitivity = 65.5;
            break;
    }

    gSensitivity_si = gSensitivity * 180 / M_PI; // LSB/°/s to LSB / rad / s 

    return gSensitivity;
}

unsigned long Imu::read_timer(void){
    return timer;
}

static inline int8_t sgn(int val) {
    if (val < 0) return -1;
    if (val==0) return 0;
    return 1;
}

void Imu::update(float tau) {
    // static unsigned long	last_time = micros();
    float dt, filter_coefficient;
    
    if(first_loop){
        first_loop = false;
        dt = 0.0;
    } else {
        dt = (float)(micros() - last_time) * 0.000001; // dt in secondes
    }
    last_time = micros();
    read_mpu_6050_data();

    gyro_x_raw -= gyro_x_cal;   //Subtract the offset calibration value from the raw gyro_x value
    gyro_y_raw -= gyro_y_cal;   //Subtract the offset calibration value from the raw gyro_y value
    gyro_z_raw -= gyro_z_cal;   //Subtract the offset calibration value from the raw gyro_z value

    gyro_x = (float) gyro_x_raw / gSensitivity_si;
    gyro_y = (float) gyro_y_raw / gSensitivity_si;
    gyro_z = (float) gyro_z_raw / gSensitivity_si;

    acc_x = (float) acc_x_raw / aSensitivity_si;
    acc_y = (float) acc_y_raw / aSensitivity_si;
    acc_z = (float) acc_z_raw / aSensitivity_si;

    angle_yaw += gyro_z * dt;
    angle_yaw = angle_yaw - (2 * M_PI) * floor(angle_yaw / (2 * M_PI));
    // to get yaw between [-M_PI; M_PI]
    if(angle_yaw > M_PI){
        angle_yaw -= 2 * M_PI;
    } 

    if (acc_z > 0) {
        angle_pitch += gyro_y * dt;
        angle_roll  += gyro_x * dt;
    } else {
        angle_pitch -= gyro_y * dt;
        angle_roll  -= gyro_x * dt;
    }

    angle_pitch -= angle_roll * sin(gyro_z * dt);
    angle_roll  += angle_pitch * sin(gyro_z * dt);

    // Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x_raw*acc_x_raw)+(acc_y_raw*acc_y_raw)+(acc_z_raw*acc_z_raw));
    angle_pitch_acc = -asin((float)acc_x_raw/acc_total_vector);      // Calculate the pitch angle based on acceleration
    angle_roll_acc  =  asin((float)acc_y_raw/acc_total_vector);      // Calculate the roll angle based on acceleration

    /*
        complementary filter
        set 0.5sec = tau = dt * A / (1 - A)
        so A = tau / (tau + dt)
    */

    // float tau = 0.5;
    // filter_coefficient = tau / (tau + dt);
    filter_coefficient = 0.9996;

    if(set_gyro_angles){
        angle_roll = angle_roll * filter_coefficient + angle_roll_acc * (1 - filter_coefficient);
        angle_pitch = angle_pitch * filter_coefficient + angle_pitch_acc * (1 - filter_coefficient);
    } else {                                                                
        //At first start
        angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
        angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
        angle_yaw = 0;
        set_gyro_angles = true;
    }

    // filter_coefficient = 0.8;
    filter_coefficient  = tau / (tau + dt);
    angle_pitch_output  = angle_pitch_output * filter_coefficient + angle_pitch * (1 - filter_coefficient);   //Take 90% of the output pitch value and add 10% of the raw pitch value
    angle_roll_output   = angle_roll_output * filter_coefficient + angle_roll * (1 - filter_coefficient);
    // angle_yaw_output   = angle_yaw_output * filter_coefficient + angle_yaw * (1 - filter_coefficient);
    angle_yaw_output    = angle_yaw;
}

float Imu::get_roll(void)  {return angle_roll_output;}
float Imu::get_pitch(void) {return angle_pitch_output;}
float Imu::get_yaw(void)   {return angle_yaw_output;}

float Imu::get_angular_velocity_x(void) {return gyro_x;}
float Imu::get_angular_velocity_y(void) {return gyro_y;}
float Imu::get_angular_velocity_z(void) {return gyro_z;}

float Imu::get_linear_acceleration_x(void) {return acc_x;}
float Imu::get_linear_acceleration_y(void) {return acc_y;}
float Imu::get_linear_acceleration_z(void) {return acc_z;}

void Imu::setup(void){
    // Init Gyroscope
    begin(MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_500);
}

void Imu::loop(void) {
    timer = millis();
    update();
}