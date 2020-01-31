#ifndef MAIN_H_   /* Include guard */
#define MAIN_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "settings.h"
#include "zero_velocity_detector.h"
#include "ZUPTaidedINS.h"

/* ZUPTaided_INS variable */
int N = WINDOWS_SIZE;
double P[9][9][WINDOWS_SIZE];
double Q[6][6];
double R1[3][3];
int H[3][9];
int Id[9][9];
double quat[4][WINDOWS_SIZE];

void navigation_equations(double ax, double ay, double az, double gx, double gy, double gz);
void step_estimation(double ax, double ay, double az, double gx, double gy, double gz);
void load_file_and_process();
void read_file(double u[][WINDOWS_SIZE]);

#endif // FOO_H_
