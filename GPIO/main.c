//
// Created by guillaume.benhamou on 26/04/2023.
//

// Librairies
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "WiringPi-master/wiringPi/wiringPi.h"
#include "WiringPi-master/wiringPi/wiringSerial.h"
#include "WiringPi-master/wiringPi/softPwm.h"

// Constantes
#define TX_LIDAR 8
#define RX_LIDAR 10
#define MOTOR_LIDAR 1

// Variables
int Serial1;

// Functions

// WiringPiTest() : Test de la librairie WiringPi
void WiringPiTest(){
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup failed.\n");
    }
    else {
        printf("WiringPi setup successful.\n");
    }
}
// enableLidar() : Active ou désactive le lidar
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
// initLidar() : Initialise le lidar
int initLidar(){
    pinMode(MOTOR_LIDAR, OUTPUT);
    enableLidar(1);
    return 0;
}
// DataRecupVerif() : Vérifie si il y a assez de données pour traiter la trame
bool DataRecupVerif(int Serial){
    if (serialDataAvail(Serial) > 80){
        return true;
    }
    else{
        return false;
    }
}
// DataRecup() : Récupère les données du lidar dans un tableau de uint8_t
uint8_t DataRecup(int Serial){
    uint8_t  data[80];
    for (int i = 0; i < 80; i++) {
        data[i] = serialGetchar(Serial);
    }
    return data[];
}
// DataTreatment() : Traite la trame du lidar
uint8_t DataTreatment(uint8_t data[80]){
    if data[0] == 0xA5{
        printf("Trame valide");
        return data;
    }
    else{
        return 0;
    }
}
// DataVerification() : Vérifie si la trame est valide
bool DataVerification(uint8_t data){

}

// readRPLidar() : Lit les données du lidar
int readRPLidar(){
    // Récupération des données.
    bool verif = DataRecupVerif(Serial1);
    uint8_t trame[80];
    if (verif == true){
        trame = DataRecup(Serial1);
    }
    else{
        return 0;
    }
    // TODO: Traitement de la trame.
    DataTreatment(trame);
    // TODO: Verification de la trame.
}

// DataCoord() : Calcul les coordonnées du point
uint8_t DataCoord(unint8_t data){

}
// AddData() : Ajoute les données dans le tableau de la map
bool AddData(uint8_t data){

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