#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"

SparkFun_ISM330DHCX myISM;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("=== ISM330DHCX I2C bring-up ===");

    Wire.begin();

    if (myISM.begin()) {
        Serial.println("IMU responded — I2C link alive, WHO_AM_I OK.");
        myISM.deviceReset();
        while (!myISM.getDeviceReset()) { delay(1); }
        Serial.println("IMU reset complete. Ready.");
    } else {
        Serial.println("No response. Check SDA/SCL, 3V3, GND, address.");
    }
}

void loop() {
}