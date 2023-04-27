//
// Created by guillaume.benhamou on 26/04/2023.
//


#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>



#define TX_LIDAR 8
#define RX_LIDAR 9
#define MOTOR_LIDAR 16

void WiringPiTest(){
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup failed.\n");
    }
}

void initLidar(){
    pinMode(MOTOR_LIDAR, OUTPUT);
    enableLidar(1);
}

void enableLidar(bool enable) {
    if (enable) {
        digitalWrite(MOTOR_LIDAR, HIGH);
        serialPutchar(serial_fd, 0xA5);
        serialPutchar(serial_fd, 0x82);
        serialPutchar(serial_fd, 0x05);
        serialPutchar(serial_fd, (uint8_t)0x00);
        serialPutchar(serial_fd, (uint8_t)0x00);
        serialPutchar(serial_fd, (uint8_t)0x00);
        serialPutchar(serial_fd, (uint8_t)0x00);
        serialPutchar(serial_fd, (uint8_t)0x00);
        serialPutchar(serial_fd, 0x22);
    } else {
        digitalWrite(MOTOR_LIDAR, LOW);
        serialPutchar(serial_fd, 0xA5);
        serialPutchar(serial_fd, 0x25);
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

int main() {
    // test of the wiringPi library
    WiringPiTest();
    // SETUP SERIAL
    Serial0 = serialOpen("/dev/ttyS0", 115200);
    Serial1 = serialOpen("/dev/ttyS1", 115200);
    // open the serial port
    SetupSerial0();
    SetupSerial1();
    // test of the pins : TX_LIDAR, RX_LIDAR, MOTOR_LIDAR
    pinMode(TX_LIDAR, OUTPUT);
    pinMode(RX_LIDAR, INPUT);
    pinMode(MOTOR_LIDAR, OUTPUT);
    digitalWrite(TX_LIDAR, HIGH);
    digitalWrite(MOTOR_LIDAR, HIGH);


    printf("TX_LIDAR : %d\n", digitalRead(TX_LIDAR));

    return 0;
}
