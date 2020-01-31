#include "main.h"

/* To compile and link in command line do the following :
 *
 * gcc -O3 main.cpp settings.cpp zero_velocity_detector.cpp ZUPTaidedINS.cpp -o OpenShoeC
 *
 */

int main(void)
{

    load_file_and_process();
    printf("Finished\n");
    return 0;
}

void load_file_and_process()
{

    /* OpenShoe algorithm variables */
    static float u_init[6][INIT_SIZE] = {0};
    static float cov_closed[9][WINDOWS_SIZE] = {0};
    float x_closed[9][WINDOWS_SIZE] = {0};
    float quat[4][WINDOWS_SIZE] = {0};
    int data_buff_count = 0;
    float u[6][WINDOWS_SIZE] = {0};
    float u_load[6][FILE_SIZE] = {0};
    bool is_Startup = 1;

    int N = WINDOWS_SIZE;
    float P[9][9][WINDOWS_SIZE] = {0};
    float Q[6][6] = {0};
    float R[3][3] = {0};
    int H[3][9] = {0};
    int Id[9][9] = {0};
    int pass = 0;
    int i = 0;
    float ax, ay, az, gx, gy, gz;

    // Loads the algorithm settings and the IMU data
    printf("Loads the algorithm settings and the IMU data\n");
    settings(u_load);

    init_filter(P, Q, R, H);
    init_vec (cov_closed, Id, P);

    for(i = 0; i < FILE_SIZE; i++) {

      ax = u_load[0][i]; ay = u_load[1][i]; az = u_load[2][i];
      gx = u_load[3][i]; gy = u_load[4][i]; gz = u_load[5][i];

      if (is_Startup == 0) {
          u[0][data_buff_count] = ax;
          u[1][data_buff_count] = ay;
          u[2][data_buff_count] = az;
          u[3][data_buff_count] = gx;
          u[4][data_buff_count] = gy;
          u[5][data_buff_count] = gz;
          data_buff_count++;
        }
        else if (is_Startup == 1) {
          u_init[0][data_buff_count] = ax;
          u_init[1][data_buff_count] = ay;
          u_init[2][data_buff_count] = az;
          u_init[3][data_buff_count] = gx;
          u_init[4][data_buff_count] = gy;
          u_init[5][data_buff_count] = gz;
          data_buff_count++;
        }

        //If we are at our defined window size we process the data.
        if(data_buff_count == WINDOWS_SIZE && is_Startup == 0) {

          int zupt[WINDOWS_SIZE] = {0}; //Maybe move that at the beginning
          zero_velocity_detector(u, zupt);

          //write_Output_zupt(zupt);

          ZUPTaidedINS(u, zupt, x_closed, cov_closed, quat, P, Q, R, H, Id, N);

          write_Output(x_closed);

          data_buff_count = 1;

          /* Saving the last computation */
          x_closed[0][0] = x_closed[0][WINDOWS_SIZE-1];
          x_closed[1][0] = x_closed[1][WINDOWS_SIZE-1];
          x_closed[2][0] = x_closed[2][WINDOWS_SIZE-1];
          x_closed[3][0] = x_closed[3][WINDOWS_SIZE-1];
          x_closed[4][0] = x_closed[4][WINDOWS_SIZE-1];
          x_closed[5][0] = x_closed[5][WINDOWS_SIZE-1];
          x_closed[6][0] = x_closed[6][WINDOWS_SIZE-1];
          x_closed[7][0] = x_closed[7][WINDOWS_SIZE-1];
          x_closed[8][0] = x_closed[8][WINDOWS_SIZE-1];

          cov_closed[0][0] = cov_closed[0][WINDOWS_SIZE-1];
          cov_closed[1][0] = cov_closed[1][WINDOWS_SIZE-1];
          cov_closed[2][0] = cov_closed[2][WINDOWS_SIZE-1];
          cov_closed[3][0] = cov_closed[3][WINDOWS_SIZE-1];
          cov_closed[4][0] = cov_closed[4][WINDOWS_SIZE-1];
          cov_closed[5][0] = cov_closed[5][WINDOWS_SIZE-1];
          cov_closed[6][0] = cov_closed[6][WINDOWS_SIZE-1];
          cov_closed[7][0] = cov_closed[7][WINDOWS_SIZE-1];
          cov_closed[8][0] = cov_closed[8][WINDOWS_SIZE-1];

          quat[0][0] = quat[0][WINDOWS_SIZE-1];
          quat[1][0] = quat[1][WINDOWS_SIZE-1];
          quat[2][0] = quat[2][WINDOWS_SIZE-1];
          quat[3][0] = quat[3][WINDOWS_SIZE-1];

          //We also need to re-use the last value of our accelerometer and gyrometer data
          //for the next pass. If we do not do that the trajectory will not be accurate.
          //(There will be big spikes).
          //That's why we use data_buff_count=1 from now on because we do not want to write
          //at the address u[i][0] the new values. We want to keep the old ones.
          //I think it something to do with the first for loop in ZUPTaidedINS and
          // res1[i] += A[i][l] * x[l][k - 1]; in Navigation_equations().
          // The algo needs to compute each values. Or if we do not re-use u[i][WINDOWS_SIZE-1]
          //in u[i][0] each time it will never be used. Causing a "break" between the new values
          // and old values each WINDOWS_SIZE.
          u[0][0] = u[0][WINDOWS_SIZE-1];
          u[1][0] = u[1][WINDOWS_SIZE-1];
          u[2][0] = u[2][WINDOWS_SIZE-1];
          u[3][0] = u[3][WINDOWS_SIZE-1];
          u[4][0] = u[4][WINDOWS_SIZE-1];
          u[5][0] = u[5][WINDOWS_SIZE-1];

          for(int k = 0; k < 9; k++) {
              for(int j = 0; j < 9; j++) {
              P[k][j][0] = P[k][j][WINDOWS_SIZE-1];
            }
          }

        }
        // We calculate the initial roll and pitch
        else if (is_Startup == 1 && data_buff_count == INIT_SIZE) {
          init_Nav_eq (u_init, x_closed, quat);
          data_buff_count = 1; //See explanations above
          is_Startup = 0;

          u[0][0] = u_init[0][INIT_SIZE - 1];
          u[1][0] = u_init[1][INIT_SIZE - 1];
          u[2][0] = u_init[2][INIT_SIZE - 1];
          u[3][0] = u_init[3][INIT_SIZE - 1];
          u[4][0] = u_init[4][INIT_SIZE - 1];
          u[5][0] = u_init[5][INIT_SIZE - 1];

        }

  }

}
