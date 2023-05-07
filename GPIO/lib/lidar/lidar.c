//
// Created by guillaume.benhamou on 03/05/2023.
//


#ifndef POUBLELLE_LIDAR_H
#define POUBLELLE_LIDAR_H

// Librairies
#include "lidar.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "../WiringPi-master/wiringPi/wiringPi.h"
#include "../WiringPi-master/wiringPi/wiringSerial.h"
#include "../WiringPi-master/wiringPi/softPwm.h"
#include "../commun-function/common_function.h"

// structures
struct syncPackets {
    // start angle [7:0]
    uint8_t startAngle;
    // S  [8:7] - start flag of a new scan
    uint8_t S;
    // start angle checksum [14:8]
    uint8_t startAngle_checksum;
};
struct Cabin{
    // distance1 [5:0]
    uint8_t dist_1;
    // delta angle 1 [5:4]
    uint8_t deltaAngle_1;
    //distance1 [13:6]
    uint8_t dist_1_bis;
    // distance2 [5:0]
    uint8_t dist_2;
    // delta angle 2 [5:4]
    uint8_t deltaAngle_2;
    // distance2 [13:6]
    uint8_t dist_3;
    // delta angle 2 bis [3:0]
    uint8_t deltaAngle_2_bis;
    // delta angle 1 bis [3:0]
    uint8_t deltaAngle_1_bis;
};
struct Trame {
    // the trame is composed of 80 bytes
    // 1 sync packets - 4 bytes
    // 16 cabins - 5 bytes each
    // sync packets

    struct  syncPackets trame_syncPackets;
    // cabins
    struct Cabin trame_cabin[16];
};



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
void enableLidar(bool enable,int Serial,int pinMotor) {
    if (enable) {
        printf("Lidar enabled.\n");
        softPwmCreate(pinMotor, 100, 100);
        serialPutchar(Serial, 0xA5);
        serialPutchar(Serial, 0x82);
        serialPutchar(Serial, 0x05);
        serialPutchar(Serial, (uint8_t)0x00);
        serialPutchar(Serial, (uint8_t)0x00);
        serialPutchar(Serial, (uint8_t)0x00);
        serialPutchar(Serial, (uint8_t)0x00);
        serialPutchar(Serial, (uint8_t)0x00);
        serialPutchar(Serial, 0x22);
    } else {
        softPwmStop(pinMotor);
        serialPutchar(Serial, 0xA5);
        serialPutchar(Serial, 0x25);
    }
}
// initLidar() : Initialise le lidar
int initLidar(int Serial, int pinMotor){
    pinMode(pinMotor, OUTPUT);
    enableLidar(1, Serial, pinMotor);
    printf("Lidar initialized.\n");
    return 0;
}
// stockPileRecupVerif() : Vérifie si il y a assez de données pour traiter la trame
bool stockPileRecupVerif(int Serial){
    if (serialDataAvail(Serial) >= 80){
        return true;
    }
    else{
        return false;
    }
}
// isSynchroByte() : Vérifie si les 2 premiers bytes sont les bytes de synchronisation
bool isSynchroByte(uint8_t syncByte1, uint8_t syncByte2){
    if (syncByte1 == 0xA5 && syncByte2 == 0x5A){
        return true;
    }
    else{
        return false;
    }
}
// DataRecup() : Récupère les données du lidar dans un tableau de uint8_t
uint8_t* DataRecup(int Serial , struct Trame DataTrame[544]){
    // faire la verification de la pile avant d'appeler la fonction
    if (stockPileRecupVerif(Serial)){
        //recup des 2 premiers bytes
        uint8_t syncByte1 = serialGetchar(Serial);
        uint8_t syncByte2 = serialGetchar(Serial);
        if (isSynchroByte(syncByte1, syncByte2)){


            //recup des 2 bytes suivants
            DataTrame->trame_syncPackets.startAngle = serialGetchar(Serial);
            //split du byte en 2: 1 bit et 7 bits
            DataTrame->trame_syncPackets.S = serialGetchar(Serial);
            DataTrame->trame_syncPackets.startAngle_checksum = serialGetchar(Serial);
            // recup des 16 cabins
            for (int i = 0; i < 16; ++i) {
                //byte1 = distance1 [5:0] + delta angle 1 [5:4]
                uint8_t byte = serialGetchar(Serial);
                DataTrame->trame_cabin[i].dist_1 = byte >> 2;
                DataTrame->trame_cabin[i].dist_1 = byte & 0x03;
                //byte2 = distance1 [13:6]
                DataTrame->trame_cabin[i].dist_1_bis = serialGetchar(Serial);
                //byte3 = distance2 [5:0] + delta angle 2 [5:4]
                byte = serialGetchar(Serial);
                DataTrame->trame_cabin[i].dist_2 = byte >> 2;
                DataTrame->trame_cabin[i].dist_2 = byte & 0x03;
                //byte4 = distance2 [13:6]
                DataTrame->trame_cabin[i].dist_3 = serialGetchar(Serial);
                //byte5 = delta angle 2 bis [3:0] + delta angle 1 bis [3:0]
                byte = serialGetchar(Serial);
                DataTrame->trame_cabin[i].deltaAngle_2_bis = byte >> 4;
                DataTrame->trame_cabin[i].deltaAngle_1_bis = byte & 0x0F;
                // print tout la data
                printf("Cabine %d\n", i+1);
                printf("startAngle: %d\n", DataTrame->trame_syncPackets.startAngle);
                printf("S: %d\n", DataTrame->trame_syncPackets.S);
                printf("startAngle_checksum: %d\n", DataTrame->trame_syncPackets.startAngle_checksum);
                printf("dist_1: %d\n", DataTrame->trame_cabin[i].dist_1);
                printf("deltaAngle_1: %d\n", DataTrame->trame_cabin[i].deltaAngle_1);
                printf("dist_1_bis: %d\n", DataTrame->trame_cabin[i].dist_1_bis);
                printf("dist_2: %d\n", DataTrame->trame_cabin[i].dist_2);
                printf("deltaAngle_2: %d\n", DataTrame->trame_cabin[i].deltaAngle_2);
                printf("dist_3: %d\n", DataTrame->trame_cabin[i].dist_3);
                printf("deltaAngle_2_bis: %d\n", DataTrame->trame_cabin[i].deltaAngle_2_bis);
                printf("deltaAngle_1_bis: %d\n", DataTrame->trame_cabin[i].deltaAngle_1_bis);
            }
            //recup des 2 bytes suivants
            return (uint8_t*)&DataTrame;
        }
        return NULL;
    }
    return NULL;
}
// DataTreatment() : Traite la trame du lidar
bool DataTreatment(){
    return 0;

}
// readRPLidar() : Lit les données du lidar
int readRPLidar(int Serial, struct Trame DataTrame[544]){
    // verification de la pile
    if(isSynchroByte(serialGetchar(Serial), serialGetchar(Serial))){
        if(stockPileRecupVerif(Serial)){
             DataRecup(Serial, DataTrame);
        }

    }
    else{
        return 1;
    }
    // recup des données
    // traitement des données



}
// DataCoord() : Calcul les coordonnées du point
uint8_t DataCoord(uint8_t data){

}
// AddData() : Ajoute les données dans le tableau de la map
bool AddData(uint8_t data){

}
// AngleDiff : Calcul la différence entre 2 angle
uint16_t AngleDiff(uint16_t angle1, uint16_t angle2){
    int diff = angle1 - angle2;
    if (diff > 180){
        diff = diff - 360;
    }
    else if (diff < -180){
        diff = diff + 360;
    }
    return diff;
}


#endif //POUBLELLE_LIDAR_H