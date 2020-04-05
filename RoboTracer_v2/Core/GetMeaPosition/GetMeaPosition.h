/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: GetMeaPosition.h
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 24-Mar-2020 17:53:43
 */

#ifndef GETMEAPOSITION_H
#define GETMEAPOSITION_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "GetMeaPosition_types.h"

/* Function Declarations */
extern void GetMeaPosition(const double PrePosition[3], double u[2], double
  position[3]);
extern void GetTruePosition(const double PrePosition[3], double u[2], double
  position[3]);

#endif

/*
 * File trailer for GetMeaPosition.h
 *
 * [EOF]
 */
