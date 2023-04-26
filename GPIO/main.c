//
// Created by guillaume.benhamou on 26/04/2023.
//

#include <../lib/WiringPi-master/wiringPi/wiringPi.h>
#include <stdio.h>
#include <time.h>



#define TX_LIDAR 8
#define RX_LIDAR 9
#define MOTOR_LIDAR 16

void WiringPiTest(){
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup failed.\n");
        return 1;
    }
}

int main() {
    // test of the wiringPi library
    WiringPiTest();

    // test of the pins : TX_LIDAR, RX_LIDAR, MOTOR_LIDAR
    pinMode(TX_LIDAR, OUTPUT);
    pinMode(RX_LIDAR, INPUT);
    pinMode(MOTOR_LIDAR, OUTPUT);
    digitalWrite(TX_LIDAR, HIGH);
    digitalWrite(MOTOR_LIDAR, HIGH);

    printf("TX_LIDAR : %d\n", digitalRead(TX_LIDAR));

    return 0;
}
