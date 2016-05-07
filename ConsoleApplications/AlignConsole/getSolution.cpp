#include "getSolution.h"
#include "dubins.h"
#include <iostream>
#include <cmath>
#include <stdint.h>

using namespace std;

// No inputs or outputs to this function, we use console for inputs and outputs
void getSolution (void) {
    // Define parameters for locations, turning, stepsize, wind vector, vt vect
    // As well as local variables for calculations
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

    cout << "Please enter the orientation at the start location (in radians): ";
    cin >> startOrientation;

    cout << "Please enter the x co-ordinate of the end location: ";
    cin >> endX;

    cout << "Please enter the y co-ordinate of the end location: ";
    cin >> endY;

    cout << "Please enter the orientation at the end location (in radians): ";
    cin >> endOrientation;

//    cout << "Here are your parameters... ";
//    cout << "\nAircraft speed: " << speed;
//    cout << "\nWind speed: " << windSpeed;
//    cout << "\nMinimum turning radius: " << radius;
//    cout << "\nStart location x co-ordinate: " << startX;
//    cout << "\nStart location y co-ordinate: " << startY;
//    cout << "\nStart location orientation: " << startOrientation;
//    cout << "\nEnd location x co-ordinate: " << endX;
//    cout << "\nEnd location y co-ordinate: " << endY;
//    cout << "\nEnd location orientation: " << endOrientation;

    double q0[3] = {startX, startY, startOrientation};
    double d[3] = {endX, endY, endOrientation};

    // Copy d into vt
    for (int i = 0; i < 3; i++) {
        vt[i] = d[i];
    }

    // We always use a stepsize of 0.1 for really accurate paths
    DubinsPath noWindPath = bestDubins(q0, d, radius, stepSize);

    //Print out segment lengths and associated flight time in seconds
    cout << "\n\nNo wind path is of type: " << getPathType(noWindPath.type);
    cout << "\n\tFirst segment has length: " << noWindPath.param[0]*radius;
    cout << "\n\t\tFlight time of: " << noWindPath.param[0]*radius/speed;
    cout << "\n\tSecond segment has length: " << noWindPath.param[1]*radius;
    cout << "\n\t\tFlight time of: " << noWindPath.param[1]*radius/speed;
    cout << "\n\tThird segment has length: " << noWindPath.param[2]*radius;
    cout << "\n\t\tFlight time of: " << noWindPath.param[2]*radius/speed;

    // Request wind condition value
    cout << "\nPlease enter the wind speed in m/s (in the positive x direction): ";
    cin >> windSpeed;

    // Calculate Ta from no wind path calculations to start
    Ta = ((noWindPath.param[0] + noWindPath.param[1] + noWindPath.param[2])*radius)/speed;
    // Start at time 0
    elapsedTime = 0.0;
    Tvt = elapsedTime;

    // Create array for 6 path types
    DubinsPath pathToVTArr[6];
    uint8_t selectedPath;
    uint32_t optimumPathSoFar = 5; // value greater than 0.1 so that first value will always set selectedpath

    // Give it a maximum limit of 300 loops
    while (elapsedTime < 30) {
        elapsedTime = elapsedTime + 0.1; // Increment calculation time value
        vt[0] = vt[0] + 0.1*(-windSpeed); // Update the x-position of the virtual target
        Tvt = elapsedTime; // Tvt update
        allDubins(pathToVTArr, q0, vt, radius, 0.1); // Get all types of dubins path
        for (int i = 0; i < 6; i++) { // Loop through path types
            Ta = ((pathToVTArr[i].param[0] + pathToVTArr[i].param[1] + pathToVTArr[i].param[2])*radius)/speed; // Calculate new Ta
            if (abs(Ta - Tvt) < 0.1) { // If we can consider the problem solved i.e. Ta and Tvt are within 0.1 second of one another
                solutionFound = true;
                if (abs(Ta - Tvt) < optimumPathSoFar) { // If this path is the best so far
                    optimumPathSoFar = abs(Ta - Tvt); // Update our best value so far
                    selectedPath = pathToVTArr[i].type; // Update selected path value
                }
            }
        }
        if (solutionFound) goto solved; // if we have a solution from one of the 6 paths, no need to keep searching
    }

    // Print out segment lengths and flight times to console
    solved:
    cout << "\n\nPath with wind is of type: " << getPathType(selectedPath);
    cout << "\n\tFirst segment has length: " << pathToVTArr[selectedPath].param[0]*radius;
    cout << "\n\t\tFlight time of: " << pathToVTArr[selectedPath].param[0]*radius/speed;
    cout << "\n\tSecond segment has length: " << pathToVTArr[selectedPath].param[1]*radius;
    cout << "\n\t\tFlight time of: " << pathToVTArr[selectedPath].param[1]*radius/speed;
    cout << "\n\tThird segment has length: " << pathToVTArr[selectedPath].param[2]*radius;
    cout << "\n\t\tFlight time of: " << pathToVTArr[selectedPath].param[2]*radius/speed;

}
