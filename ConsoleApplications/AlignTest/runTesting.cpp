#include "calculatePaths.h"


using namespace std;

void runTesting(void)
{
    // Variables
    double q0[3] = {0, 0, 0};
    double d[3] = {0, 0, 0};
    int windspeed = 0;

/**
  * Windspeed varies from -9 to 9 at intervals of 1
  *
  * Orientations vary from 0 to 6.3 (2*pi) at intervals of 0.79 (pi/4)
  *
  * d[x] varies from 0 to 90 in intervals of 15
  *
  * d[y] varies from 0 to 90 in intervals of 15
  */

    // Constants
    uint32_t speed = 18; //18 m/s for this UAV
    uint32_t r = 25; //Radius
    double stepsize = 0.1; //This is constant

    for (int dx = 0; dx < 91; dx += 15) {
        for (int dy = 0; dy < 91; dy += 15) {
            for (int wspd = -9; wspd < 10; wspd++) {
                for (int dO = 0; dO < 8; dO++) {
                    for (int qO = 0; qO < 8; qO++) {
                        d[0] = dx;
                        d[1] = dy;
                        d[2] = dO;
                        q0[2] = qO;
                        calculatePaths (r, q0, d, stepsize, speed, wspd);
                    }
                }
            }
        }
    }


//    for (int startO = 0; startO < 8; startO++) {
//        q0[2] = startO*0.785;
//        for (int endO = 0; endO < 8; endO++) {
//            d[2] = endO*0.785;
//            for (int endX = 0; endX < 8; endX++) {
//                d[0] = endX*10;
//                for (int endY = 0; endY < 8; endY++) {
//                    d[1] = endY*10;
//                    calculatePaths (25, q0, d, 0.1, 18, -9);
//                }
//            }
//        }
//    }
}
