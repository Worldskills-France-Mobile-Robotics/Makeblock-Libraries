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

float Imu::get_roll(void)  {return ypr[2];}
float Imu::get_pitch(void) {return ypr[1];}
float Imu::get_yaw(void)   {return ypr[0];}

float Imu::get_angular_velocity_x(void) {return gyro.x / (65.5 * 180 / M_PI);}
float Imu::get_angular_velocity_y(void) {return gyro.y / (65.5 * 180 / M_PI);}
float Imu::get_angular_velocity_z(void) {return gyro.z / (65.5 * 180 / M_PI);}

float Imu::get_linear_acceleration_x(void) {return aaReal.x / 16384.0;}
float Imu::get_linear_acceleration_y(void) {return aaReal.y / 16384.0;}
float Imu::get_linear_acceleration_z(void) {return aaReal.z / 16384.0;}

void Imu::setup(void){
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

bool Imu::loop(void) {
    bool ret = false;
    if (!dmpReady) return ret;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        ret = true;
        timer = millis();
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGyro(&gyro, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        // mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    }

    return ret;
}