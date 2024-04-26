#include "MPU9250.h"
#include "MadgwickAHRS.c"

MPU9250 mpu;

struct Quat {
    float x;
    float y;
    float z;
    float w;
} quat;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    mpu.verbose(false);
}

void loop() {
    if (mpu.update()) {
        // Read sensor data
        float ax = mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
        float ay = mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
        float az = mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY;
        float gx = mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;
        float gy = mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;
        float gz = mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;
        
        // Update orientation using MadgwickAHRS algorithm
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        
        // Get quaternion values
        quat.x = q0;
        quat.y = q1;
        quat.z = q2;
        quat.w = q3;

        // Print quaternion values
        print_quaternion();
    }
}

void print_quaternion() {
    Serial.print("Quaternion: ");
    Serial.print("x: ");
    Serial.print(quat.x, 4);
    Serial.print(", y: ");
    Serial.print(quat.y, 4);
    Serial.print(", z: ");
    Serial.print(quat.z, 4);
    Serial.print(", w: ");
    Serial.println(quat.w, 4);
}