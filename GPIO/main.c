//
// Created by guillaume.benhamou on 26/04/2023.
//


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "../GPIO/WiringPi-master/wiringPi/wiringPi.h"
#include "WiringPi-master/wiringPi/wiringSerial.h"
#include "../GPIO/WiringPi-master/wiringPi/softPwm.h"


#define TX_LIDAR 8
#define RX_LIDAR 10
#define MOTOR_LIDAR 12

int Serial0;
int Serial1;


void WiringPiTest(){
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup failed.\n");
    }
    else {
        printf("WiringPi setup successful.\n");
    }
}
void enableLidar(bool enable) {
    if (enable) {
        softPwmCreate(MOTOR_LIDAR, 100, 100);
        // measure of the delay of one command sent to the lidar
        unsigned long t0 = micros();
        serialPutchar(Serial1, 0xA5);
        printf("Time to send the command : %ld\n", micros() - t0);
        serialPutchar(Serial1, 0x82);
        serialPutchar(Serial1, 0x05);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, 0x22);
    } else {
        softPwmStop(MOTOR_LIDAR);
        serialPutchar(Serial1, 0xA5);
        serialPutchar(Serial1, 0x25);
    }
}

int initLidar(){
    pinMode(MOTOR_LIDAR, OUTPUT);
    enableLidar(1);
    return 0;
}



int main() {
    // test of the wiringPi library
    WiringPiTest();
    // SETUP SERIAL
    Serial1 = serialOpen("/dev/ttyS1", 115200);
    // open the serial port
    initLidar();
    // close the serial port
    serialClose(Serial0);
    return 0;
}
