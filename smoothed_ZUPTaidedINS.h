#ifndef SMOOTHEDZUPTAIDEDINS_H_   /* Include guard */
#define SMOOTHEDZUPTAIDEDINS_H_

#include "settings.h"
#include "ZUPTaidedINS.h"

#define N_MAT 9

extern struct simdata_struct simdata;



void smoothed_ZUPTaidedINS(float u[][WINDOWS_SIZE], int zupt[], float x[][WINDOWS_SIZE], float cov_smoothed[][WINDOWS_SIZE], float P_smooth[][9][WINDOWS_SIZE], float dx[][WINDOWS_SIZE], float dx_smooth[][WINDOWS_SIZE], float P[][9][WINDOWS_SIZE]);
void update_cov_smoothed(float P_smooth[][9][WINDOWS_SIZE], float P[][9][WINDOWS_SIZE], float A[][9], float P_timeupd[][9][WINDOWS_SIZE], float cov_smoothed[][WINDOWS_SIZE] , int n);
void state_update(float dx_smooth[9][WINDOWS_SIZE], float dx[9][WINDOWS_SIZE], float A[][9], float dx_timeupd[9][WINDOWS_SIZE], int n);
void update_deviation_estimate(float dx[][WINDOWS_SIZE], float K[][3], float x[][WINDOWS_SIZE], int n);
void update_state_cov_zupt_smoothed (float P[][9][WINDOWS_SIZE], int Id[][9], float K[][3], int H[][9], float cov[][WINDOWS_SIZE], int k);
void update_state_cov_smoothed (float F[][9], float P[][9][WINDOWS_SIZE], float G[][6], float Q[][6], float cov[][WINDOWS_SIZE], int k);
void compute_A(float A[][9], float P[][9][WINDOWS_SIZE], float Fn[][9][WINDOWS_SIZE], float P_timeupd[][9][WINDOWS_SIZE], int k);
void inv_mat_9(float I[][9], float mat[][9][WINDOWS_SIZE], int k);




#endif // FOO_H_
