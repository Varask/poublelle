//
// Created by guillaume.benhamou on 26/04/2023.
//

// Librairies
#include "lib/WiringPi-master/wiringPi/wiringPi.h"
#include "lib/WiringPi-master/wiringPi/wiringSerial.h"
#include "lib/WiringPi-master/wiringPi/softPwm.h"
#include "lib/lidar/lidar.h"

// -------- MAIN -----------
int main() {
    // test of the wiringPi library
    WiringPiTest();
    // SETUP SERIAL
    Serial1 = serialOpen("/dev/ttyS1", 115200);
    // SETUP LIDAR
    initLidar(Serial1, MOTOR_LIDAR);
    // READ LIDAR
    readRPLidar(Serial1);
    return 0;
}