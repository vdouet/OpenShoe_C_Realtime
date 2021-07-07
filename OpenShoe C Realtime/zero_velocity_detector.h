#ifndef ZERO_VELOCITY_DETECTOR_H_   /* Include guard */
#define ZERO_VELOCITY_DETECTOR_H_

#include "settings.h"
//#include "main.h"

#define LENGTH(x)  (sizeof(x) / sizeof((x)[0]))

extern struct simdata_struct simdata;
#define LEN simdata.Window_size

void zero_velocity_detector(float u[][WINDOWS_SIZE], int zupt[]);
void GLRT_f(float u[][WINDOWS_SIZE], float *T);
void ARE_f(float u[][WINDOWS_SIZE], float *T);
int max(float arr[], int n);

#endif // FOO_H_
