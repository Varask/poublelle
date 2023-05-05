//
// Created by guillaume.benhamou on 03/05/2023.
//

#ifndef POUBLELLE_LIDAR_H
#define POUBLELLE_LIDAR_H

// Librairies
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

// Constantes
#define TX_LIDAR 8
#define RX_LIDAR 10
#define MOTOR_LIDAR 12

// Variables
int Serial1;
bool dataGathering = false;

// structures
struct syncPackets{
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

struct Trame{
    // the trame is composed of 80 bytes
    // 1 sync packets - 4 bytes
    // 16 cabins - 5 bytes each
    // sync packets
    struct syncPackets trame_syncPackets;
    // cabins
    struct Cabin trame_cabin[16];

};
// Fonctions
void WiringPiTest();
void enableLidar(bool enable,int Serial,int pinMotor);
int initLidar(int Serial, int pinMotor);
bool stockPileRecupVerif(int Serial);
bool AddData(uint8_t data);
uint8_t* DataRecup(int Serial);
uint8_t* DataTreatment(uint8_t data[80]);
bool isSynchroByte(uint8_t syncByte1, uint8_t syncByte2);
int readRPLidar(int Serial);
uint8_t DataCoord(uint8_t data);

void startDataGathering();
void stopDataGathering();

#endif //POUBLELLE_LIDAR_H
