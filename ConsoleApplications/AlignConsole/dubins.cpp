
// Copyright (c) 2008-2010, Andrew Walker
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Extended by Alex Cash 2016
// Comments will indicate where additions have been made by Alex Cash
#define _USE_MATH_DEFINES

#include "dubins.h"
#include <cmath>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <string>

using namespace std;

#define EPSILON (10e-10)

#define LSL (0)
#define LSR (1)
#define RSL (2)
#define RSR (3)
#define RLR (4)
#define LRL (5)

#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

const int DIRDATA[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};

#define UNPACK_INPUTS(alpha, beta)     \
    double sa = sin(alpha);            \
    double sb = sin(beta);             \
    double ca = cos(alpha);            \
    double cb = cos(beta);             \
    double c_ab = cos(alpha - beta);   \

#define PACK_OUTPUTS(outputs)       \
    outputs[0]  = t;                \
    outputs[1]  = p;                \
    outputs[2]  = q;

/* Added by Alex Cash */
/* If we only want to get one Dubins path back */
/* Inputs:  Start location
            End location
            Min. turn radius
            Resolution for path readings
 */
/* Output: Dubinspath structure defined in dubins.h*/
DubinsPath bestDubins (double q0[3], double q1[3], double r, double stepSize) {

    DubinsPath path;
    dubins_init( q0, q1, r, &path);
    return path;

}

/* Added by Alex Cash */
/* If we want all types returned. We update the array of dubins paths called x */
/* Inputs:  Start location
            End location
            Min. turn radius
            Resolution for path readings
 */
/* Output: Array of Dubinspath structures defined in dubins.h*/
void allDubins (DubinsPath *x, double q0[3], double q1[3], double r, double stepSize) {

    dubins_init_all(x, q0, q1, r);


}

/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
double fmodr( double x, double y)
{
    return x - y*floor(x/y);
}

double mod2pi( double theta )
{
    return fmodr( theta, 2 * M_PI );
}

int dubins_init( double q0[3], double q1[3], double rho, DubinsPath* path )
{
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double D = sqrt( dx * dx + dy * dy );
    double d = D / rho;
    if( rho <= 0. ) {
        return EDUBBADRHO;
    }
    if( (fabs(dx) < EPSILON) && (fabs(dy) < EPSILON) ) {
        // if the configurations are colocated, theta has no clear definition
        // TODO test if you can get away with letting theta = 0
        return EDUBCOCONFIGS;
    }
    double theta = mod2pi(atan2( dy, dx ));
    double alpha = mod2pi(q0[2] - theta);
    double beta  = mod2pi(q1[2] - theta);
    for( int i = 0; i < 3; i ++ ) {
        path->qi[i] = q0[i];
    }
    return dubins_init_normalised( alpha, beta, d, rho, path );
}

/* Added by Alex Cash */
/* Extended to initialise all of the qi fields in the path structures */
/* Inputs:  Start location
            End location
            Min. turn radius
            Resolution for path readings
            Array of Dubinspath structures defined in dubins.h
 */

int dubins_init_all(DubinsPath *x, double q0[3], double q1[3], double rho)
{
    // Calculate the normalise positions
    double dx = q1[0] - q0[0];
    double dy = q1[1] - q0[1];
    double D = sqrt( dx * dx + dy * dy );
    double d = D / rho;
    if( rho <= 0. ) {
        return EDUBBADRHO;
    }
    if( (fabs(dx) < EPSILON) && (fabs(dy) < EPSILON) ) {
        return EDUBCOCONFIGS;
    }
    double theta = mod2pi(atan2( dy, dx ));
    double alpha = mod2pi(q0[2] - theta);
    double beta  = mod2pi(q1[2] - theta);
    // Loop through all paths to load in initial location configs
    for( int i = 0; i < 3; i ++ ) {
        x[0].qi[i] = q0[i];
        x[1].qi[i] = q0[i];
        x[2].qi[i] = q0[i];
        x[3].qi[i] = q0[i];
        x[4].qi[i] = q0[i];
        x[5].qi[i] = q0[i];
    }
    // Call the actual calculation and processing function
    return dubins_init_normalised_all(x, alpha, beta, d, rho);
}

void dubins_LSL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp0 = d+sa-sb;
    double tmp2 = 2 + (d*d) -(2*c_ab) + (2*d*(sa - sb));
    if( tmp2 >= 0 && tmp0 >= 0) {
        double tmp1 = atan( (cb-ca)/tmp0 );
        double t = mod2pi(-alpha + tmp1 );
        double p = sqrt( tmp2 );
        double q = mod2pi(beta -   tmp1 );
        PACK_OUTPUTS(outputs);
    }
}

void dubins_RSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp0 = d-sa+sb;
    double tmp2 = 2 + (d*d) -(2*c_ab) + (2*d*(sb-sa));
    if( tmp2 >= 0 && tmp0 >= 0) {
        double tmp1 = atan( (ca-cb)/tmp0 );
        double t = mod2pi( alpha - tmp1 );
        double p = sqrt( tmp2 );
        double q = mod2pi( -beta + tmp1 );
        PACK_OUTPUTS(outputs);
    }
}

void dubins_LSR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = -2 + (d*d) + (2*c_ab) + (2*d*(sa+sb));
    if( tmp1 >= 0 ) {
        double p    = sqrt( tmp1 );
        double tmp2 = atan( (-ca-cb)/(d+sa+sb) ) - atan(-2.0/p);
        double t    = mod2pi(-alpha + tmp2);
        double q    = mod2pi( -mod2pi(beta) + tmp2 );
        PACK_OUTPUTS(outputs);
    }
}

void dubins_RSL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp1 = (d*d) -2 + (2*c_ab) - (2*d*(sa+sb));
    if( tmp1 > 0 ) {
        double p    = sqrt( tmp1 );
        double tmp2 = atan( (ca+cb)/(d-sa-sb) ) - atan(2.0/p);
        double t    = mod2pi(alpha - tmp2);
        double q    = mod2pi(beta - tmp2);
        PACK_OUTPUTS(outputs);
    }
}

void dubins_RLR( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp_rlr = (6. - d*d + 2*c_ab + 2*d*(sa-sb)) / 8.;
    if( fabs(tmp_rlr) < 1) {
        double p = acos( tmp_rlr );
        double t = mod2pi(alpha - atan2( ca-cb, d-sa+sb ) + mod2pi(p/2.));
        double q = mod2pi(alpha - beta - t + mod2pi(p));
        PACK_OUTPUTS( outputs );
    }
}

void dubins_LRL( double alpha, double beta, double d, double* outputs )
{
    UNPACK_INPUTS(alpha, beta);

    double tmp_lrl = (6. - d*d + 2*c_ab + 2*d*(- sa + sb)) / 8.;

    if( fabs(tmp_lrl) < 1) {
        double p = mod2pi(acos(tmp_lrl));
        double t = mod2pi(-alpha - atan2( ca-cb, d+sa-sb ) + p/2.);
        double q = mod2pi(mod2pi(beta) - alpha -t + mod2pi(p));
        PACK_OUTPUTS( outputs );
    }

}

int dubins_init_normalised( double alpha,
                            double beta,
                            double d,
                            double rho,
                            DubinsPath* path)
{
    path->rho = rho;

    // Take the precaution of initialising all results
    double results[6][4];
    for( int i = 0; i < 6; i++ ) {
        results[i][0] = HUGE_VAL;//INFINITY;
        results[i][1] = HUGE_VAL;//INFINITY;
        results[i][2] = HUGE_VAL;//INFINITY;
    }

    // For each trajectory class, find the solution
    dubins_LSL( alpha, beta, d, results[LSL] );
    dubins_LSR( alpha, beta, d, results[LSR] );
    dubins_RSL( alpha, beta, d, results[RSL] );
    dubins_RSR( alpha, beta, d, results[RSR] );
    dubins_RLR( alpha, beta, d, results[RLR] );
    dubins_LRL( alpha, beta, d, results[LRL] );

    // Generate the total costs for each trajectory class
    for(int i = 0; i < 6; i++)
    {
        results[i][3] = 0;
        for(int j = 0; j < 3; j++) {
            //assert( results[i][j] >= 0. );
            //std::cout << results[i][j] << " ";
            results[i][3] += results[i][j];
        }
        //std::cout << std::endl;
    }

    // Extract the best cost path
    int bestType = 0;
    double minCost = results[0][3];
    for(int i = 1; i < 6; i++) // Updated by Alex Cash to assess all paths, including LSL paths
    {
        //std::cout << i << " " << results[i][3] << std::endl;
        if( results[i][3] < minCost ) {
            minCost = results[i][3];
            bestType = i;
        }
    }
    //std::cout << "best = " << bestType << std::endl;

    // Copy the results into the output structure
    path->type = bestType;
    for(int i = 0; i < 3; i++)
    {
        path->param[i] = results[bestType][i];
    }
    return 0;
}

/* Added by Alex Cash */
/* Function to process all paths as opposed to just the shortest */
/* Inputs:  Normalised path parameters
            Array of Dubinspath structures defined in dubins.h
 */
/* Output: return 0 for success */

int dubins_init_normalised_all(DubinsPath *x,
                            double alpha,
                            double beta,
                            double d,
                            double rho)
{

    // Fill all path rho fields
    x[0].rho = rho;
    x[1].rho = rho;
    x[2].rho = rho;
    x[3].rho = rho;
    x[4].rho = rho;
    x[5].rho = rho;

    // Take the precaution of initialising all results
    double results[6][4];
    for( int i = 0; i < 6; i++ ) {
        results[i][0] = HUGE_VAL;//INFINITY;
        results[i][1] = HUGE_VAL;//INFINITY;
        results[i][2] = HUGE_VAL;//INFINITY;
    }

    // For each trajectory class, find the solution
    dubins_LSL( alpha, beta, d, results[LSL] );
    dubins_LSR( alpha, beta, d, results[LSR] );
    dubins_RSL( alpha, beta, d, results[RSL] );
    dubins_RSR( alpha, beta, d, results[RSR] );
    dubins_RLR( alpha, beta, d, results[RLR] );
    dubins_LRL( alpha, beta, d, results[LRL] );

    // Loop through all path types
    for (int i = 0; i < 6; i++) {
        x[i].type = i; //Set the type for the path
        for (int j = 0; j < 3; j++) { // Loop through each segment of each path
             x[i].param[j] = results[i][j]; // Store the results in the param variables
        }
    }
    return 0;
}

/* Added by Alex Cash */
/* Path to just return a string of the path type based on input number */
/* Inputs: path type integer */
string getPathType(int pathType) {
    string outStr;
    switch (pathType) {
        case 0:
            outStr = "LSL";
            break;
        case 1:
            outStr = "LSR";
            break;
        case 2:
            outStr = "RSL";
            break;
        case 3:
            outStr = "RSR";
            break;
        case 4:
            outStr = "RLR";
            break;
        case 5:
            outStr = "LRL";
            break;
        default:
            outStr = "NUL";
            break;
    }
    return outStr;
}
