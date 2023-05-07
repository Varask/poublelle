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
    int SerialLidar= serialOpen("/dev/ttyS1", 115200);
    int SerialMotorGauche = serialOpen("/dev/ttyUSB0", 115200);
    int SerialMotorDroit = serialOpen("/dev/ttyUSB1", 115200);
    // SETUP LIDAR
    initLidar(SerialLidar, MOTOR_LIDAR);
    // READ LIDAR
    readRPLidar(SerialLidar);
    return 0;
}