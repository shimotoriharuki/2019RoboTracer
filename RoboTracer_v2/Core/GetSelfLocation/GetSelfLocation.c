/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: GetSelfLocation.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 24-Mar-2020 17:34:25
 */

/* Include Files */
#include <math.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "GetSelfLocation.h"
#include "randn.h"
#include "GetSelfLocation_data.h"

/* Custom Source Code */
/* Author: SHIMOTORI, Haruki */

/* Function Definitions */

/*
 * --------------------Init------------------%
 * Arguments    : const double PrePosition[3]
 *                const double PrePt[9]
 *                double PreZt
 *                const double velo[2]
 *                const double HatPosition[3]
 *                double EstPosition[3]
 *                double EstPt[9]
 *                double *ObsZt
 * Return Type  : void
 */
void GetSelfLocation(const double PrePosition[3], const double PrePt[9], double
                     PreZt, const double velo[2], const double HatPosition[3],
                     double EstPosition[3], double EstPt[9], double *ObsZt)
{
  double dSr;
  double dSl;
  double u_idx_0;
  double u_idx_1;
  double At[9];
  double Wt[6];
  double dv0[4];
  int i0;
  int i1;
  double b_At[9];
  double b_Wt[6];
  double HatPt[9];
  static const signed char b[3] = { 0, 0, 1 };

  double Kt[3];
  static const signed char a[3] = { 0, 0, 1 };

  /* ---------------------- Calclation start----------------------------% */
  /*  Calclate dS & dTh */
  dSr = (velo[1] * Tred + 2.0 * velo[0]) / 2.0 * dt;
  dSl = (-velo[1] * Tred + 2.0 * velo[0]) / 2.0 * dt;
  u_idx_0 = (dSr + dSl) / 2.0;
  u_idx_1 = (dSr - dSl) / Tred;

  /*  The process noise covariance matrix */
  /* ----------------------- Forecast step----------------------% */
  /*  Calclation At & Wt */
  At[0] = 1.0;
  At[3] = 0.0;
  dSr = PrePosition[2] + u_idx_1 / 2.0;
  dSl = sin(dSr);
  At[6] = -u_idx_0 * dSl;
  At[1] = 0.0;
  At[4] = 1.0;
  dSr = cos(dSr);
  At[7] = u_idx_0 * dSr;
  At[2] = 0.0;
  At[5] = 0.0;
  At[8] = 1.0;
  Wt[0] = dSr;
  Wt[3] = -u_idx_0 / 2.0 * dSl;
  Wt[1] = dSl;
  Wt[4] = u_idx_0 / 2.0 * dSr;
  Wt[2] = 0.0;
  Wt[5] = 1.0;

  /*  Calclation estmation errors covariance matrix */
  dSl = u_idx_0 * u_idx_0;
  dSr = u_idx_1 * u_idx_1;
  dv0[0] = ErrerParameter[0] * dSl + ErrerParameter[1] * dSr;
  dv0[3] = ErrerParameter[2] * dSl + ErrerParameter[3] * dSr;
  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      b_At[i0 + 3 * i1] = (At[i0] * PrePt[3 * i1] + At[i0 + 3] * PrePt[1 + 3 *
                           i1]) + At[i0 + 6] * PrePt[2 + 3 * i1];
    }

    dSl = Wt[i0 + 3];
    b_Wt[i0] = Wt[i0] * dv0[0] + dSl * 0.0;
    b_Wt[i0 + 3] = Wt[i0] * 0.0 + dSl * dv0[3];
    for (i1 = 0; i1 < 3; i1++) {
      HatPt[i0 + 3 * i1] = (b_At[i0] * At[i1] + b_At[i0 + 3] * At[i1 + 3]) +
        b_At[i0 + 6] * At[i1 + 6];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (i1 = 0; i1 < 3; i1++) {
      At[i0 + 3 * i1] = b_Wt[i0] * Wt[i1] + b_Wt[i0 + 3] * Wt[i1 + 3];
    }
  }

  for (i0 = 0; i0 < 9; i0++) {
    HatPt[i0] += At[i0];
  }

  /* ------------ Update step^^^^^^^^^^% */
  /*  Get Robot's angle for gyro or geomagnetism */
  /* Calclation angle by gyro */
  /*  This is rewritten to the calclation IMU angle program     */
  /*
  dSr = sqrt(Qt);
  dSl = randn();
  dSl *= dSr;
  if (dSr < 0.0) {
    dSl = rtNaN;
  }
  */
  *ObsZt = PreZt + (velo[1] * dt);

  /*  Covariance of observation residuals */
  dSr = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    dSr += ((0.0 * HatPt[3 * i0] + 0.0 * HatPt[1 + 3 * i0]) + HatPt[2 + 3 * i0])
      * (double)b[i0];
  }

  dSl = dSr + Qt;

  /*  Calclation Kalman constant */
  /*  Transpose matrix */
  /*  Calclation estimation position */
  dSr = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    Kt[i0] = ((HatPt[i0] * 0.0 + HatPt[i0 + 3] * 0.0) + HatPt[i0 + 6]) / dSl;
    dSr += (double)a[i0] * HatPosition[i0];
  }

  dSr = *ObsZt - dSr;
  EstPosition[0] = HatPosition[0] + Kt[0] * dSr;
  EstPosition[1] = HatPosition[1] + Kt[1] * dSr;
  EstPosition[2] = HatPosition[2] + Kt[2] * dSr;

  /*  Updata estmation errors covariance matrix */
  memset(&At[0], 0, 9U * sizeof(double));
  At[0] = 1.0;
  At[4] = 1.0;
  At[8] = 1.0;
  for (i0 = 0; i0 < 3; i0++) {
    b_At[i0] = At[i0] - Kt[i0] * 0.0;
    b_At[i0 + 3] = At[i0 + 3] - Kt[i0] * 0.0;
    b_At[i0 + 6] = At[i0 + 6] - Kt[i0];
    for (i1 = 0; i1 < 3; i1++) {
      EstPt[i0 + 3 * i1] = (b_At[i0] * HatPt[3 * i1] + b_At[i0 + 3] * HatPt[1 +
                            3 * i1]) + b_At[i0 + 6] * HatPt[2 + 3 * i1];
    }
  }
}

/*
 * File trailer for GetSelfLocation.c
 *
 * [EOF]
 */
