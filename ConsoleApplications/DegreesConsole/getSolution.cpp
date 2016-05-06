#include "getSolution.h"
#include "dubins.h"
#include <iostream>
#include <cmath>
#include <stdint.h>

using namespace std;

void getSolution (void) {
    double radius;
    double startX, startY, startOrientation, endX, endY, endOrientation;
    double Ta, Tvt, elapsedTime;
    double vt[3];
    double stepSize = 0.1; //Always use 0.1
    double speed;
    double windSpeed;
    bool solutionFound = false;

    cout << "Please enter the speed of the aircraft in m/s: " << endl;
    cin >> speed;

    cout << "Please enter the min turning radius of the aircraft in m: ";
    cin >> radius;

    cout << "Please enter the x co-ordinate of the start location: ";
    cin >> startX;

    cout << "Please enter the y co-ordinate of the start location: ";
    cin >> startY;

    cout << "Please enter the orientation at the start location (in degrees clockwise from North): ";
    cin >> startOrientation;

    cout << "Please enter the x co-ordinate of the end location: ";
    cin >> endX;

    cout << "Please enter the y co-ordinate of the end location: ";
    cin >> endY;

    cout << "Please enter the orientation at the end location (in degrees clockwise from North): ";
    cin >> endOrientation;


    //Convert degrees to radians, with correct orientations
    // input * pi/180
    startOrientation = startOrientation - 90; // dubins program starts at east = 0 radians for some reason
    startOrientation = 360 - startOrientation; //dubins program measures it the wrong way round for some unknown reason...
    startOrientation = startOrientation * M_PI/180; //convert to radians

    endOrientation = endOrientation - 90;
    endOrientation = 360 - endOrientation;
    endOrientation = endOrientation * M_PI/180;

    if (startOrientation >= 2*M_PI) {
        startOrientation = startOrientation - 2*M_PI;
    }

    if (endOrientation >= 2*M_PI) {
        endOrientation = endOrientation - 2*M_PI;
    }

    cout << "\n\nq0 orientation is: " << startOrientation;
    cout << "\n\nq1 orientation is: " << endOrientation << "\n\n";

    double q0[3] = {startX, startY, startOrientation};
    double d[3] = {endX, endY, endOrientation};

    // Copy d into vt
    for (int i = 0; i < 3; i++) {
        vt[i] = d[i];
    }



    DubinsPath noWindPath = bestDubins(q0, d, radius, stepSize);
    // uint32_t Ta

    cout << "\n\nNo wind path is of type: " << getPathType(noWindPath.type);
    cout << "\n\n\tFirst segment has length: " << noWindPath.param[0]*radius;
    cout << "\n\t\t\tFlight time of: " << noWindPath.param[0]*radius/speed*1000 << " ms";
    cout << "\n\n\tSecond segment has length: " << noWindPath.param[1]*radius;
    cout << "\n\t\t\tFlight time of: " << noWindPath.param[1]*radius/speed*1000 << " ms";
    cout << "\n\n\tThird segment has length: " << noWindPath.param[2]*radius;
    cout << "\n\t\t\tFlight time of: " << noWindPath.param[2]*radius/speed*1000 << " ms";

    //system("pause");

    cout << "\nPlease enter the wind speed in m/s (in the positive x direction): ";
    cin >> windSpeed;


    Ta = ((noWindPath.param[0] + noWindPath.param[1] + noWindPath.param[2])*radius)/speed;
    elapsedTime = 0.0;
    Tvt = elapsedTime;

    DubinsPath pathToVTArr[6];
    uint8_t selectedPath;
    uint32_t optimumPathSoFar = 5; // value greater than 0.05 so that first value will always set selectedpath

    while (elapsedTime < 30) {
        elapsedTime = elapsedTime + 0.1;
        vt[0] = vt[0] + 0.1*(-windSpeed); //Update the x-position of the virtual target
        Tvt = elapsedTime;
        allDubins(pathToVTArr, q0, vt, radius, 0.1); //get all types of dubins path
        for (int i = 0; i < 6; i++) {
            Ta = ((pathToVTArr[i].param[0] + pathToVTArr[i].param[1] + pathToVTArr[i].param[2])*radius)/speed;
            if (abs(Ta - Tvt) < 0.1) { //if we can consider the problem solved
                solutionFound = true;
                if (abs(Ta - Tvt) < optimumPathSoFar) {
                    optimumPathSoFar = abs(Ta - Tvt);
                    selectedPath = pathToVTArr[i].type;
                }

            }
        }
        if (solutionFound) goto solved; // if we have a solution from one of the 6 paths, no need to keep searching
    }

    solved:
    cout << "\n\nPath with wind is of type: " << getPathType(selectedPath);
    cout << "\n\tFirst segment has length: " << pathToVTArr[selectedPath].param[0]*radius;
    cout << "\n\t\t\tFlight time of: " << pathToVTArr[selectedPath].param[0]*radius/speed*1000 << " ms";
    cout << "\n\n\tSecond segment has length: " << pathToVTArr[selectedPath].param[1]*radius;
    cout << "\n\t\t\tFlight time of: " << pathToVTArr[selectedPath].param[1]*radius/speed*1000 << " ms";
    cout << "\n\n\tThird segment has length: " << pathToVTArr[selectedPath].param[2]*radius;
    cout << "\n\t\t\tFlight time of: " << pathToVTArr[selectedPath].param[2]*radius/speed*1000 << " ms";

}
