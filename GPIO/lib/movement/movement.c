//
// Created by guillaume.benhamou on 05/05/2023.
//

#ifndef POUBLELLE_MOVEMENT_H
#define POUBLELLE_MOVEMENT_H

// Librairies
#include "stdbool.h"
#include "stdint.h"
#include "stdio.h"
#include "../WiringPi-master/wiringPi/wiringPi.h"
#include "../WiringPi-master/wiringPi/wiringSerial.h"
#include "../WiringPi-master/wiringPi/softPwm.h"
#include "../commun-function/common_function.h"
// Constantes
#define USBSERIAL0 "/dev/ttyUSB0"
#define USBSERIAL1 "/dev/ttyUSB1"
#define USBSERIAL_SPEED 1000000

// Variables
uint8_t position_Motor1 = 0;
uint8_t position_Motor2 = 0;

// fonctions
void setupSerial(char *serialPort,long serialSpeed){
    int setupValue = 0xFF;
    serialPutchar(USBSERIAL0,setupValue);
}
#endif //POUBLELLE_MOVEMENT_H