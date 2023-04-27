//
// Created by guillaume.benhamou on 26/04/2023.
//


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "../GPIO/WiringPi-master/wiringPi/wiringPi.h"
#include "../GPIO/WiringPi-master/wiringPi/wiringPi.h"
#include "WiringPi-master/wiringPi/wiringSerial.h"


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
        analogWrite(MOTOR_LIDAR, 1023);
        serialPutchar(Serial1, 0xA5);
        serialPutchar(Serial1, 0x82);
        serialPutchar(Serial1, 0x05);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, (uint8_t)0x00);
        serialPutchar(Serial1, 0x22);
    } else {
        analogWrite(MOTOR_LIDAR, 0);
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
    Serial0 = serialOpen("/dev/ttyS0", 115200);
    Serial1 = serialOpen("/dev/ttyS1", 115200);
    // open the serial port
    initLidar();


    // test of the pins : TX_LIDAR, RX_LIDAR, MOTOR_LIDAR
    pinMode(TX_LIDAR, OUTPUT);
    pinMode(RX_LIDAR, INPUT);
    pinMode(MOTOR_LIDAR, OUTPUT);
    digitalWrite(TX_LIDAR, HIGH);
    digitalWrite(MOTOR_LIDAR, HIGH);


    printf("TX_LIDAR : %d\n", digitalRead(TX_LIDAR));


    // close the serial port
    serialClose(Serial0);
    return 0;
}
