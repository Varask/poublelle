#ifndef lidar_h
#define lidar_h

#include "Arduino.h"
#define NBMESURESMAX 220

#define NUL -32768


// LIDAR 
#define LIDARPWMPIN 2 
#define LIDAR Serial1

// Dimensions du robot
#define DEMILARGEURROBOT 150
#define DEMILONGUEURROBOT 150

//Timer
int16_t getX();
int16_t getY();
float getThetaRad();

//Défintion de la structure Point
typedef struct {
 union {
  struct {
   int16_t x;
   int16_t y;
  };

  int16_t coordonnees[2];

  uint8_t bytes[4];
 };
} Point;

const Point POINTNUL = {NUL, NUL};

uint8_t getNbPoints();
Point * getPointsLidarPointer();
Point * getPointsCartePointer();
int16_t getPointLidar(uint8_t n, uint8_t axe);
int16_t getPointCarte(uint8_t n, uint8_t axe);
void initLidar();
void enableLidar(bool enable);
bool isNewScan();
void clearNewScan();
bool readLidar();

#define NBITERATIONSSYNCHRO 3
#define NBMESURESCABINES 32
#define UNTOURQ6 (360 << 6)
#define LIDARTIMEOUT 120  // 66 15Hz

#define POSITIONLIDARX 0
#define POSITIONLIDARY 0
//#define ANGLELIDAR -PISURDEUX16
#define ANGLELIDAR 0.0

#define POSITIONSROBOTXMIN -DEMILARGEURROBOT
#define POSITIONSROBOTXMAX DEMILARGEURROBOT
#define POSITIONSROBOTYMIN -DEMILONGUEURROBOT
#define POSITIONSROBOTYMAX DEMILONGUEURROBOT



#endif
