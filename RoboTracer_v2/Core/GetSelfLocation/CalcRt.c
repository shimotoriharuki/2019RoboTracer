/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: CalcRt.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 24-Mar-2020 14:05:57
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "GetSelfLocation.h"
#include "CalcRt.h"

/* Function Definitions */

/*
 * Arguments    : const double param[4]
 *                const double u[2]
 *                double out[4]
 * Return Type  : void
 */
void CalcRt(const double param[4], const double u[2], double out[4])
{
  double out_tmp;
  double b_out_tmp;
  out_tmp = u[0] * u[0];
  b_out_tmp = u[1] * u[1];
  out[0] = param[0] * out_tmp + param[1] * b_out_tmp;
  out[2] = 0.0;
  out[1] = 0.0;
  out[3] = param[2] * out_tmp + param[3] * b_out_tmp;
}

/*
 * File trailer for CalcRt.c
 *
 * [EOF]
 */
