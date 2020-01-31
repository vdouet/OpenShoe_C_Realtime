#ifndef ZUPTAIDEDINS_H_   /* Include guard */
#define ZUPTAIDEDINS_H_

#include <stdbool.h>
#include "settings.h"
//#include "main.h"

#define LENGTH_2D(x)  (sizeof(x)[0] / sizeof((x)[0][0]))
extern struct simdata_struct simdata;

void ZUPTaidedINS(float u[][WINDOWS_SIZE], int zupt[], float x_h[][WINDOWS_SIZE], float cov[][WINDOWS_SIZE], float quat[][WINDOWS_SIZE], float P[][9][WINDOWS_SIZE], float Q[][6], float R[][3], int H[][9], int Id[][9], int N);
void init_filter(float P[][9][WINDOWS_SIZE], float Q[][6], float R[][3], int H[][9]);
void init_vec(float cov[][WINDOWS_SIZE], int Id[][9], float P[][9][WINDOWS_SIZE]);
void init_Nav_eq(float u[][INIT_SIZE], float x_h[][WINDOWS_SIZE], float quat[][WINDOWS_SIZE]);
void Navigation_equations(float x[][WINDOWS_SIZE], float u[], float q[][WINDOWS_SIZE], int k);
void state_matrix(float q[][WINDOWS_SIZE], float u[], float F[][9], float G[][6], int k);
void update_state_cov(float F[][9], float P[][9][WINDOWS_SIZE], float G[][6], float Q[][6], float cov[][WINDOWS_SIZE], int k);
void kalman_filter_gain(float K[][3], float P[][9][WINDOWS_SIZE], int H[][9], float R[][3], int k);
void comp_internal_states(float x_h[][WINDOWS_SIZE], float dx[], float q[][WINDOWS_SIZE], int k);
void update_state_cov_zupt(float P[][9][WINDOWS_SIZE], int Id[][9], float K[][3], int H[][9], float cov[][WINDOWS_SIZE], int k);
void Rt2b(float ang[], float R[][3]);
void dcm2q(float R[][3], float quat[][WINDOWS_SIZE], int k);
void q2dcm(float R[][3], float q[][WINDOWS_SIZE], int k);
void inv_mat(float mat_inv[][3], float mat[][3]);


#endif // FOO_H_
