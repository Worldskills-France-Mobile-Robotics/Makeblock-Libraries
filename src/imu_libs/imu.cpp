#include "imu.hpp"

Imu::Imu(uint8_t port) : MePort(port){
    timer = millis() + 40;
    device_address = MPU6050_DEFAULT_ADDRESS;

    #ifdef ME_PORT_DEFINED
    _AD0 = s1;
    _INT = s2;
    #endif // ME_PORT_DEFINED

    // Set _AD0 to the ground
    // https://www.i2cdevlib.com/forums/topic/414-freezing-problem/
    MePort::dWrite1(LOW);
}

unsigned long Imu::read_timer(void){
    return timer;
}

void Imu::begin(uint8_t accel_config, uint8_t gyro_config) {
    aSensitivity = set_aSensitivity(accel_config);
    gSensitivity = set_gSensitivity(gyro_config);

    Wire.begin();

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // Serial.print(F("DMP Initialization failed (code "));
        // Serial.print(devStatus);
        // Serial.println(F(")"));
        dmpReady = false;
    }
}

float Imu::get_aSensitivity(void){
    return aSensitivity;
}

float Imu::get_aSensitivity_si(void){
    return aSensitivity_si;
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

float Imu::get_gSensitivity_si(void){
    return gSensitivity_si;
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

float Imu::get_roll(void)  {return ypr[2];}
float Imu::get_pitch(void) {return ypr[1];}
float Imu::get_yaw(void)   {return ypr[0];}

float Imu::get_angular_velocity_x(void) {return gyro.x / get_gSensitivity_si();}
float Imu::get_angular_velocity_y(void) {return gyro.y / get_gSensitivity_si();}
float Imu::get_angular_velocity_z(void) {return gyro.z / get_gSensitivity_si();}

float Imu::get_linear_acceleration_x(void) {return aaReal.x / get_aSensitivity_si();}
float Imu::get_linear_acceleration_y(void) {return aaReal.y / get_aSensitivity_si();}
float Imu::get_linear_acceleration_z(void) {return aaReal.z / get_aSensitivity_si();}

void Imu::setup(void){
    begin(MPU6050_ACCEL_FS_2, MPU6050_GYRO_FS_500);
}

bool Imu::loop(void) {
    bool ret = false;
    if (!dmpReady) return ret;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(lastPacket)) {
        ret = true;
        timer = millis();
        // Get the Quaternion from sensor
        mpu.dmpGetQuaternion(&q, lastPacket);
        // Normalize the quaternium we need this
        // For the equations to be valid
        q.normalize();

        mpu.dmpGetAccel(&aa, lastPacket);
        mpu.dmpGetGyro(&gyro, lastPacket);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    }

    return ret;
}