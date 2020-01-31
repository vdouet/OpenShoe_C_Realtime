#include "zero_velocity_detector.h"

void zero_velocity_detector(float u[][WINDOWS_SIZE], int zupt[]) {

  int i,k;
  int idx = 0;
  
  // Allocate memory
  int W = simdata.Window_size;
  int N = WINDOWS_SIZE;
  float T[N-W+1];
  //memset(T, 0, W * sizeof(float));
  memset(T, 0, sizeof(T));

  // Run the desired detector type. Each detector return a vector with their
  // calculated test statistics T.
  switch (simdata.detector_type) {

      case GLRT:
          GLRT_f(u, T);
          break;

      case MV:
          //MV_f(u, T);
          break;

      case MAG:
          //MAG_f(u, T);
          break;

      case ARE:
          ARE_f(u, T);
          break;

      default:
          printf("The choosen detector type not recognized. The GLRT detector is used");
          GLRT_f(u, T);
          break;
  }

  int T_size = N-W+1;
  int zupt_size = N;

  // Check if the test statistics T are below the detector threshold. If so,
  // chose the hypothesis that the system has zero velocity
  for(k = 0; k < T_size; k++) {
    if(T[k] < simdata.gamma) {
      for(i = 0; i < W; i++) {
        zupt[k+i] = 1;
      }
    }
  }

  // Fix the edges of the detector statistics
  int size = T_size + floor(W/2)*2;
  float T_n[size];
  memset(T_n, 0, W * sizeof(float));

  for(i = 0; i < floor(W/2); i++) {
      T_n[i] = max(T, T_size);
      idx++;
  }

  memcpy(T_n + idx, T, sizeof(T));
  idx += T_size;
  for(i = 0; i < floor(W/2); i++) {
      T_n[i + idx] = max(T, T_size);
  }

}

void GLRT_f(float u[][WINDOWS_SIZE], float *T) {

  float g = simdata.g;
  float sigma2_a = simdata.sigma_a*simdata.sigma_a;
  float sigma2_g = simdata.sigma_g*simdata.sigma_g;
  int W = simdata.Window_size;
  float ya_m[3] = {0};
  float tmp[3] = {0};
  float a = 0, b = 0;
  int i,k,l;

  int N = WINDOWS_SIZE;

  for(k = 0; k < N-W+1; k++) {

    for(i = k; i < k+W-2; i++) {

      ya_m[0] = u[0][i] + u[0][i+1] + u[0][i+2];
      ya_m[1] = u[1][i] + u[1][i+1] + u[1][i+2];
      ya_m[2] = u[2][i] + u[2][i+1] + u[2][i+2];

    }
    ya_m[0] /= W;
    ya_m[1] /= W;
    ya_m[2] /= W;

    for(l = k; l <= k+W-1; l++) {

      tmp[0] = u[0][l] - (g * ya_m[0] / sqrt(ya_m[0] * ya_m[0] + ya_m[1] * ya_m[1] + ya_m[2] * ya_m[2]));
      tmp[1] = u[1][l] - (g * ya_m[1] / sqrt(ya_m[0] * ya_m[0] + ya_m[1] * ya_m[1] + ya_m[2] * ya_m[2]));
      tmp[2] = u[2][l] - (g * ya_m[2] / sqrt(ya_m[0] * ya_m[0] + ya_m[1] * ya_m[1] + ya_m[2] * ya_m[2]));

      a = (u[3][l] * u[3][l] + u[4][l] * u[4][l] + u[5][l] * u[5][l]) / sigma2_g; //a = u(4:6,l)'*u(4:6,l)/sigma2_g

      b = (tmp[0] * tmp[0] + tmp[1] * tmp[1] + tmp[2] * tmp[2]) / sigma2_a; //b = tmp'*tmp/sigma2_a
      T[k] = T[k] + a + b; // T(k)+u(4:6,l)'*u(4:6,l)/sigma2_g+tmp'*tmp/sigma2_a;


    }
  }

  for(i = 0; i < N-W+1; i++) {
    T[i] /= W;
  }



}

void ARE_f(float u[][WINDOWS_SIZE], float *T) {

  float g = simdata.g;
  float sigma2_g = simdata.sigma_g*simdata.sigma_g;
  int W = simdata.Window_size;
  int i = 0, k = 0, l = 0;

  int N = WINDOWS_SIZE;

  for(k = 0; k < N-W+1; k++) {
    for(l = k; l <= k+W-1; l++) {
      T[k] = T[k] + sqrt(u[3][l] * u[3][l] + u[4][l] * u[4][l] + u[5][l] * u[5][l]) * sqrt(u[3][l] * u[3][l] + u[4][l] * u[4][l] + u[5][l] * u[5][l]); //T(k)+norm(u(4:6,l))^2;
    }
  }

  for(i = 0; i < N-W+1; i++) {
    T[i] /= (sigma2_g*W);
  }

}


int max(float arr[], int n)
{
    int i;
    int max = arr[0];

    for (i = 1; i < n; i++)  {
      if (arr[i] > max)
          max = arr[i];
    }

    return max;
}
