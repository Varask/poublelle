//
// Created by guillaume.benhamou on 26/04/2023.
//


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "WiringPi-master/wiringPi/wiringPi.h"
#include "WiringPi-master/wiringPi/wiringSerial.h"
#include "WiringPi-master/wiringPi/softPwm.h"


#define TX_LIDAR 8
#define RX_LIDAR 10
#define MOTOR_LIDAR 1

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

    // read the serial port for 10 seconds
    unsigned long t0 = millis();
    while (millis() - t0 < 10000) {
        readRPLidar();
    }
    return 0;
}

// DataRecup() : Récupère les données du lidar dans un tableau de uint8_t
uint8_t DataRecup(){
    int dataDispo = serialDataAvail(Serial1);
    uint8_t data[dataDispo];

    if (dataDispo > 0) {
        for (int i = 0; i < dataDispo; i++) {
            data[i] = serialGetchar(Serial1);
        }
    }

    // print the data received
    for (int i = 0; i < dataDispo; i++) {
        printf("%d \n", data[i]);
    }
    return data[];
}

int DataTreatment(uint8_t* data){
}
bool DataVerification(uint8_t data){

}
uint8_t DataCoord(unint8_t data){

}
bool AddData(uint8_t data){

}
int readRPLidar(){
    // Récupération des données.
    uint8_t trame = DataRecup();
    // TODO: Traitement de la trame.
    DataTreatment(trame);
    // TODO: Verification de la trame.
    if (DataVerification(trame) == true){
        // TODO: Calcul des coordonnées.
        // TODO: Stockage des données.
        AddData(DataCoord(trame)));
    }

}

