#include "calculatePaths.h"
#include "dubins.h"
#include <iostream>
#include <fstream>
#include <cmath>

using namespace std;

void calculatePaths (uint32_t radius, double q0[3], double d[3], double stepsize,
                     uint32_t speed, int windSpeed) {

    double Ta, Tvt, elapsedTime;
    double vt[3];
    bool solutionFound = false;
    string typeOfPath;

    for (int i = 0; i < 3; i++) {
        vt[i] = d[i];
    }


    DubinsPath zeroWind = bestDubins(q0, d, radius, stepsize);
    Ta = ((zeroWind.param[0] + zeroWind.param[1] + zeroWind.param[2])*radius)/speed;
    elapsedTime = 0.0;
    Tvt = elapsedTime;

    DubinsPath pathToVTArr[6];
    int selectedPath;
    uint32_t optimumPathSoFar = 5; // value greater than 0.05 so that first value will always set selectedpath
    cout << "\nEntering Loop ";
    while (elapsedTime < 30) {
        cout << "\nEntered Loop ";
        elapsedTime = elapsedTime + 0.1;
        vt[0] = vt[0] + 0.1*(-windSpeed); //Update the x-position of the virtual target
        Tvt = elapsedTime;
        allDubins(pathToVTArr, q0, vt, radius, stepsize); //get all types of dubins path
        for (int i = 0; i < 6; i++) {
            cout << "\nEntering inner Loop: " << i;
            Ta = ((pathToVTArr[i].param[0] + pathToVTArr[i].param[1] + pathToVTArr[i].param[2])*radius)/speed;
            if (abs(Ta - Tvt) < 0.05) { //if we can consider the problem solved
                cout << "\nCondition satisfied " << i;
                solutionFound = true;
                if (abs(Ta - Tvt) < optimumPathSoFar) {
                    optimumPathSoFar = abs(Ta - Tvt);
                    selectedPath = pathToVTArr[i].type;
                    cout << "\nSetting selected path as " << selectedPath;
                }

            }
        }
        cout << "\nDoWeGOTO? " << solutionFound;
        if (solutionFound) goto solved; // if we have a solution from one of the 6 paths, no need to keep searching
    }

    solved:
    cout << "\nGone to GOTO lol ";
    ofstream myfile;
    myfile.open ("test.txt", ios_base::app);
    cout << "\n File opened...";
    if (elapsedTime >= 30) {
        cout << "\nelapsedTime too high lol ";
        myfile << " ERROR calculating path n\n\n";
    } else {
        cout << "\nPrinting to file ";
        myfile << "Start: \t\t" << q0[0] << ", " << q0[1] << ", " << q0[2] << "\n";
        cout << "\nPrinted start ";
        myfile << "End: \t\t" << d[0] << ", " << d[1] << ", " << d[2] << "\n";
        cout << "\nPrinted end ";
        myfile << "Vt:  \t\t\t" << vt[0] << ", " << vt[1] << ", " << vt[2] << "\n";
        cout << "\nPrinted vt ";
        //typeOfPath = getPathType(selectedPath);
        myfile << "Path type: \t" << selectedPath << "\n";
        cout << "\nPrinted path type";
        myfile << "Section 1: \t" << pathToVTArr[selectedPath].param[0]*radius << "\n";
        myfile << "Section 2: \t" << pathToVTArr[selectedPath].param[1]*radius << "\n";
        myfile << "Section 3: \t" << pathToVTArr[selectedPath].param[2]*radius << "\n";
        myfile << "Total: \t" << (pathToVTArr[selectedPath].param[0] + pathToVTArr[selectedPath].param[0] + pathToVTArr[selectedPath].param[0])*radius << "\n\n\n";
        cout << "\nPrinted total... ";
    }
}


