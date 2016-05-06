#include <iostream>
#include "dubins.h"
#include "findSuitablePath.h"


using namespace std;

void getPaths (void) {
    uint32_t radius;
    double startX, startY, startOrientation, endX, endY, endOrientation;

    /* cout << "Please enter the speed of the aircraft in m/s: " << endl;
    cin >> speed;

    cout << "Please enter the wind speed in m/s: " << endl;
    cin >> windSpeed;

    cout << "Please enter the min turning radius of the aircraft in m: ";
    cin >> radius;

    cout << "Please enter the x co-ordinate of the start location: ";
    cin >> startX;

    cout << "Please enter the y co-ordinate of the start location: ";
    cin >> startY;

    cout << "Please enter the orientation at the start location: ";
    cin >> startOrientation;

    cout << "Please enter the x co-ordinate of the end location: ";
    cin >> endX;

    cout << "Please enter the y co-ordinate of the end location: ";
    cin >> endY;

    cout << "Please enter the orientation at the end location: ";
    cin >> endOrientation;

    cout << "Here are your parameters... ";
    cout << "\nAircraft speed: " << speed;
    cout << "\nWind speed: " << windSpeed;
    cout << "\nMinimum turning radius: " << radius;
    cout << "\nStart location x co-ordinate: " << startX;
    cout << "\nStart location y co-ordinate: " << startY;
    cout << "\nStart location orientation: " << startOrientation;
    cout << "\nEnd location x co-ordinate: " << endX;
    cout << "\nEnd location y co-ordinate: " << endY;
    cout << "\nEnd location orientation: " << endOrientation; */



    startX = startY = endY = 0;
    endX = 60;
    startOrientation = 1.57;
    endOrientation = 4.71;
    radius = 25;


    double q0[3] = {startX, startY, startOrientation};
    double d[3] = {endX, endY, endOrientation};

    // We always use a stepsize of 0.1 for really accurate paths
    // TODO determine if this is necessary/needs changing
    double stepSize = 0.1;

    //noWindPath is the path that the UAV would take if there were no wind
    // i.e. no difference between ground relative and air relative paths
    DubinsPath noWindPath = bestDubins(q0, d, radius, stepSize);
    // uint32_t Ta

    cout << "\n\nNo wind path is of type: " << getPathType(noWindPath.type);
    cout << "\n\tFirst segment has length: " << noWindPath.param[0]*radius;
    cout << "\n\tSecond segment has length: " << noWindPath.param[1]*radius;
    cout << "\n\tThird segment has length: " << noWindPath.param[2]*radius;


    /*
    // Get all paths
    DubinsPath allPaths[6];
    allDubins(allPaths, q0, d, radius, stepSize);

    for (int i = 0; i < 6; i++) {
        cout << "\n\nPath " << i;
        cout << "\n\tType: " << getPathType(allPaths[i].type);
        cout << "\n\tSegment 1 length: " << allPaths[i].param[0]*radius;
        cout << "\n\tSegment 2 length: " << allPaths[i].param[1]*radius;
        cout << "\n\tSegment 3 length: " << allPaths[i].param[2]*radius;
        cout << "\n\tTotal length: " << (allPaths[i].param[0] + allPaths[i].param[1] + allPaths[i].param[2])*radius;
    } */

    addWind(radius, q0, d, stepSize);

}

void addWind(uint32_t radius, double q0[3], double d[3], double stepsize) {
    uint32_t speed, windSpeed;
    double Ta, Tvt, elapsedTime;
    double vt[3];

    for (int i = 0; i < 3; i++) {
        vt[i] = d[i];
    }


    // virtual target (vt) starts at desired endpoint (d)
    // Ta is time taken for uav to fly the suggested path
    // Tvt is the time taken for the vt to travel to its location - usually time elapsed

    // q0 is our start point
    // d is our endpoint (ground position)
    // the resultant vt is the ground relative point we need to command the aircraft to fly to given the wind
    // condition that will result in the aircraft reaching d
    cout << "\n\nPlease enter the speed in m/s of the aircraft: ";
    cin >> speed;
    cout << "\nPlease enter the wind speed in m/s (in the positive x direction): ";
    cin >> windSpeed;

    DubinsPath zeroWind = bestDubins(q0, d, radius, stepsize);
    Ta = ((zeroWind.param[0] + zeroWind.param[1] + zeroWind.param[2])*radius)/speed;
    elapsedTime = 0.0;
    Tvt = elapsedTime;

    DubinsPath pathToVT, pathToVTArr[6];
    uint8_t selectedPath;

    /* while (((Ta - Tvt) > 0.05) | ((Ta-Tvt) < -0.05)) {
        elapsedTime = elapsedTime + 0.1;
        vt[0] = vt[0] + 0.1*(-windSpeed); //Update the x-position of the virtual target
        Tvt = elapsedTime;
        pathToVT = bestDubins(q0, vt, radius, stepsize);
        Ta = ((pathToVT.param[0] + pathToVT.param[1] + pathToVT.param[2])*radius)/speed;

    } */


    while (elapsedTime < 30) {
        elapsedTime = elapsedTime + 0.1;
        vt[0] = vt[0] + 0.1*(-windSpeed); //Update the x-position of the virtual target
        Tvt = elapsedTime;
        allDubins(pathToVTArr, q0, vt, radius, stepsize); //get all types of dubins path
        for (int i = 0; i < 6; i++) {
            Ta = ((pathToVTArr[i].param[0] + pathToVTArr[i].param[1] + pathToVTArr[i].param[2])*radius)/speed;
            if (((Ta - Tvt) < 0.05) && ((Ta-Tvt) > -0.05)) {
                selectedPath = pathToVTArr[i].type;
                goto solutionFound;
            }
        }
    }

    solutionFound:

    cout << "\nSelected path is: " << getPathType(selectedPath);
    cout << "\n\nFinal vt co-ords are : " << vt[0] << ", " << vt[1] << ", " << vt[2];


}

















