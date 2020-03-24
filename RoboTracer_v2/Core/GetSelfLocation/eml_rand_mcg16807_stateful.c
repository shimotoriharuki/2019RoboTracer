/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: eml_rand_mcg16807_stateful.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 24-Mar-2020 14:05:57
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "GetSelfLocation.h"
#include "eml_rand_mcg16807_stateful.h"
#include "GetSelfLocation_data.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void eml_rand_mcg16807_stateful_init(void)
{
  b_state = 1144108930U;
}

/*
 * File trailer for eml_rand_mcg16807_stateful.c
 *
 * [EOF]
 */
