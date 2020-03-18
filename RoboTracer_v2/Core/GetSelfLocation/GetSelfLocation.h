/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: GetSelfLocation.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 17-Mar-2020 14:06:13
 */

#ifndef GETSELFLOCATION_H
#define GETSELFLOCATION_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "GetSelfLocation_types.h"

/* Function Declarations */
extern void GetSelfLocation(const double PrePosition[3], const double PrePt[9],
  double PreZt, const double velo[2], double EstPosition[3], double EstPt[9],
  double *ObsZt);

#endif

/*
 * File trailer for GetSelfLocation.h
 *
 * [EOF]
 */
