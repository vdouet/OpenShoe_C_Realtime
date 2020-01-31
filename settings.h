#ifndef SETTINGS_H_   /* Include guard */
#define SETTINGS_H_

#define pi    M_PI
#define GLRT  0
#define MV    1
#define MAG   2
#define ARE   3
#define WINDOWS_SIZE 3
#define INIT_SIZE 20
#define FILE_SIZE 3+51052
#define BUFF 0


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

void settings(float u[][FILE_SIZE]);
float gravity(float lambda, float h);
void load_dataset(float u[][FILE_SIZE]);
void write_Output(float x[9][WINDOWS_SIZE]);
void write_Output_zupt(int zupt[WINDOWS_SIZE]);

struct simdata_struct {
    float   altitude;
    float   latitude;
    float   init_heading;
    float   init_pos[3];
    float   sigma_acc[3];
    float   sigma_gyro[3];
    float   sigma_vel[3];
    float   sigma_a;
    float   sigma_g;
    int     Window_size;
    int     gamma;
    float   g;
    float   Ts;
    int     detector_type;
    float   sigma_initial_pos[3];
    float   sigma_initial_vel[3];
    float   sigma_initial_att[3];
};




#endif // FOO_H_
