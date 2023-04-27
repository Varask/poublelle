#include "lidar.h"


//int16_t getX(){
//  return 1350;
//}
//int16_t getY(){
//  return 1325;
//}
//float getThetaRad(){
//  return PI / 2;
//}

Point pointsLidar[NBMESURESMAX];
//int16_t pointsx[NBMESURESMAX];
//int16_t pointsy[NBMESURESMAX];
Point pointsCarte[NBMESURESMAX];
//int16_t pointsCarte[2][NBMESURESMAX];

uint8_t nbPoints = 0;

bool lidarOk = false;
uint8_t nbSyncroLidar = 0;
bool newScan = false;

bool isNewScan() {
  return newScan;
}

void clearNewScan() {
  newScan = false;
}

uint8_t getNbPoints() {
  return nbPoints;
}

Point * getPointsLidarPointer() {
  return pointsLidar;
}

Point * getPointsCartePointer() {
  return pointsCarte;
}

int16_t getPointLidar(uint8_t n, uint8_t axe) {
  return pointsLidar[n].coordonnees[axe];
}

int16_t getPointCarte(uint8_t n, uint8_t axe) {
  return pointsCarte[n].coordonnees[axe];
}

void initLidar() {
  LIDAR.begin(115200);
  pinMode(LIDARPWMPIN, OUTPUT);

  for (uint8_t i = 0; i < NBMESURESMAX; i++) {
    pointsLidar[i] = POINTNUL;
    pointsCarte[i] = POINTNUL;
  }
  enableLidar(true);
}



void enableLidar(bool enable) {
  if (enable) {
    digitalWrite(LIDARPWMPIN, true);
    LIDAR.write(0xA5);
    LIDAR.write(0x82);
    LIDAR.write(0x05);
    LIDAR.write((uint8_t)0x00);
    LIDAR.write((uint8_t)0x00);
    LIDAR.write((uint8_t)0x00);
    LIDAR.write((uint8_t)0x00);
    LIDAR.write((uint8_t)0x00);
    LIDAR.write(0x22);
  } else {
    digitalWrite(LIDARPWMPIN, false);
    LIDAR.write(0xA5);
    LIDAR.write(0x25);
    nbSyncroLidar = 0;
    lidarOk = false;
  }
}


bool readLidar() {
  static uint8_t n = 0;                     
  static uint8_t checksum;                  
  static uint8_t sum = 0;
  static uint16_t startAngleQ6;
  static uint16_t diffAngleQ6;
  static uint16_t oldStartAngleQ6 = 0;
  static int32_t oldAngleBrutQ6;
  static uint32_t last = 0;
  static uint8_t deltaAnglesQ3[NBMESURESCABINES];
  static uint16_t distances[NBMESURESCABINES];
  static uint8_t m = 0;
  static uint8_t s = 0;
  static uint16_t j = 0;
  int32_t diffAngleTotalQ6;

  while (LIDAR.available()) {
    uint8_t current = LIDAR.read();

    switch (n) {

      case 0:                                                                             // Début de réception de l'en-tête
        if (current >> 4 == 0xA) {
          checksum = current & 0xF;
          n = 1;
        }
        break;

      case 1:
        if (current >> 4 == 0x5) {
          checksum |= current << 4;
          n = 2;
        } else
          n = 0;
        break;

      case 2:
        sum ^= current;
        startAngleQ6 = current;
        n = 3;
        break;

      case 3:
        sum ^= current;
        startAngleQ6 |= (current & 0x7F) << 8;
        // bool start = current >> 7;                                                               // Fin de réception de l'en-tête
        // if(start)
        //  Serial.println("start");

        if (nbSyncroLidar < NBITERATIONSSYNCHRO) {                                                  // Ne pas calculer pendant la synchronisation ou sans les cabines
          nbSyncroLidar++;
          n = 4;
          break;
        }

        diffAngleQ6 = startAngleQ6 - oldStartAngleQ6;                                      // Calculer l'angle entre deux mesures de référence
        if (oldStartAngleQ6 > startAngleQ6)
          diffAngleQ6 += UNTOURQ6;

        diffAngleTotalQ6 = 0;
        for (uint8_t i = 0; i < NBMESURESCABINES; i++) {

          int32_t angleBrutQ6 = (oldStartAngleQ6 + diffAngleTotalQ6 / NBMESURESCABINES) % UNTOURQ6; // Calculer l'angle non compensé
          diffAngleTotalQ6 += diffAngleQ6;

          if (oldAngleBrutQ6 > angleBrutQ6) {                                                       // Détection du passage par zéro de l'angle non compensé
            nbPoints = j;
            j = 0;
            uint32_t now = millis();
            if (now - last > LIDARTIMEOUT)
              lidarOk = false;
            else
              lidarOk = true;
            last = now;
            newScan = true;
          }
          oldAngleBrutQ6 = angleBrutQ6;

          if (distances[i] && j < NBMESURESMAX) {                                           // Si la lecture est valide et si il reste de la place dans les tableaux
            float angle = angleBrutQ6 - (deltaAnglesQ3[i] << 3);                                   // Calculer l'angle compensé
            angle = angle * (2 * PI) / UNTOURQ6 + ANGLELIDAR;                                                // Remise à l'échelle de l'angle //TODO

            // Passage du plan du lidar au plan du robot
            int16_t xpoint = POSITIONLIDARX + distances[i] * sin(angle);
            int16_t ypoint = POSITIONLIDARY + distances[i] * cos(angle);

            pointsLidar[j] = {xpoint, ypoint};           // Enregistrement cartésien

            if (xpoint > POSITIONSROBOTXMAX || xpoint < POSITIONSROBOTXMIN ||
                ypoint > POSITIONSROBOTYMAX || ypoint < POSITIONSROBOTYMIN) {         // Si la lecture est hors du robot

              pointsLidar[j] = {xpoint, ypoint};                                      // Enregistrement cartésien

              // Passage du plan du robot au plan de la carte
              int16_t xrobot = getX();
              int16_t yrobot = getY();
              float theta = getThetaRad();
              pointsCarte[j] = { int16_t(xrobot + (xpoint * cos(theta) - ypoint * sin(theta))),
                                 int16_t(yrobot + (xpoint * sin(theta) + ypoint * cos(theta))) 
                               };
              j++;
            }
          }

        }
        oldStartAngleQ6 = startAngleQ6;

        n = 4;
        break;

      default:                                                                            // Début de réception des cabines
        sum ^= current;
        switch (m) {
          case 0:
            deltaAnglesQ3[s] = (current & 0b11) << 4;
            distances[s] = current >> 2;
            m = 1;
            break;
          case 1:
            distances[s] |= current << 6;
            m = 2;
            break;
          case 2:
            deltaAnglesQ3[s + 1] = (current & 0b11) << 4;
            distances[s + 1] = current >> 2;
            m = 3;
            break;
          case 3:
            distances[s + 1] |= current << 6;
            m = 4;
            break;
          case 4:
            deltaAnglesQ3[s] |= current & 0b1111;
            deltaAnglesQ3[s + 1] |= current >> 4;
            m = 0;
            s += 2;
            if (s == NBMESURESCABINES) {
              if (sum != checksum) {
                //if (init == NBITERATIONSSYNCHRO)
                //  TXDEBUG.println("checksum lidar");
                nbSyncroLidar--;                                                // Ne pas faire les calculs pour ces cabines
              }
              n = 0;
              s = 0;
              sum = 0;
            }
            break;
        }
        break;

    }
  }

  return lidarOk;
}
