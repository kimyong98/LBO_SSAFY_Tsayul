#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(100);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(1000);
        }
    }

    mpu.selectFilter(QuatFilterSel::MADGWICK);
    mpu.verbose(true);
    mpu.calibrateAccelGyro();
    delay(1000);
    mpu.calibrateMag();
    mpu.verbose(false);
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_quaternion();
            /*
            송신 코드
            */
            prev_ms = millis();
        }
    }
}

void print_quaternion() {
    Serial.print(mpu.getQuaternionX());
    Serial.print(",");
    Serial.print(mpu.getQuaternionY());
    Serial.print(",");
    Serial.print(mpu.getQuaternionZ());
    Serial.print(",");
    Serial.println(mpu.getQuaternionW());
}