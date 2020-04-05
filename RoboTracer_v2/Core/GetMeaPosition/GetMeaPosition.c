/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: GetMeaPosition.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 24-Mar-2020 17:53:43
 */

/* Include Files */
#include <math.h>
#include "GetMeaPosition.h"
#include "randn.h"
#include "GetMeaPosition_data.h"

/* Function Definitions */

/*
 * This is rewritten to the robot odometry program
 * Arguments    : const double PrePosition[3]
 *                double u[2]
 *                double position[3]
 * Return Type  : void
 */
void GetMeaPosition(const double PrePosition[3], double u[2], double position[3])
{
  double Rt_idx_3;
  double Rt_idx_0_tmp;
  double Rt_idx_0;
  Rt_idx_3 = u[0] * u[0];
  Rt_idx_0_tmp = u[1] * u[1];
  Rt_idx_0 = ErrerParameter[0] * Rt_idx_3 + ErrerParameter[1] * Rt_idx_0_tmp;
  Rt_idx_3 = ErrerParameter[2] * Rt_idx_3 + ErrerParameter[3] * Rt_idx_0_tmp;

  /*  the process noise covariance matrix */
  u[0] += randn() * sqrt(Rt_idx_0);
  u[1] += randn() * sqrt(Rt_idx_3);
  Rt_idx_3 = PrePosition[2] + u[1] / 2.0;
  position[0] = PrePosition[0] + u[0] * cos(Rt_idx_3);
  position[1] = PrePosition[1] + u[0] * sin(Rt_idx_3);
  position[2] = PrePosition[2] + u[1];
}

void GetTruePosition(const double PrePosition[3], double u[2], double position[3])
{
  double Rt_idx_3;
  //double Rt_idx_0_tmp;
  //double Rt_idx_0;
  //Rt_idx_3 = u[0] * u[0];
  //Rt_idx_0_tmp = u[1] * u[1];
  //Rt_idx_0 = 0 * Rt_idx_3 + 0 * Rt_idx_0_tmp;
  //Rt_idx_3 = 0 * Rt_idx_3 + 0 * Rt_idx_0_tmp;

  /*  the process noise covariance matrix */
  //u[0] += randn() * sqrt(Rt_idx_0);
  //u[1] += randn() * sqrt(Rt_idx_3);
  Rt_idx_3 = PrePosition[2] + u[1] / 2.0;
  position[0] = PrePosition[0] + u[0] * cos(Rt_idx_3);
  position[1] = PrePosition[1] + u[0] * sin(Rt_idx_3);
  position[2] = PrePosition[2] + u[1];
}

/*
 * File trailer for GetMeaPosition.c
 *
 * [EOF]
 */
