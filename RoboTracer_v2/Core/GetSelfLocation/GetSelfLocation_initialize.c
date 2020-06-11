/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: GetSelfLocation_initialize.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 24-Mar-2020 17:34:25
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "GetSelfLocation.h"
#include "GetSelfLocation_initialize.h"
#include "eml_rand_shr3cong_stateful.h"
#include "eml_rand_mcg16807_stateful.h"
#include "eml_rand.h"
#include "eml_randn.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "GetSelfLocation_data.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void GetSelfLocation_initialize(void)
{
  rt_InitInfAndNaN(8U);
  state_not_empty_init();
  method_not_empty_init();
  dt = 0.01;
  Tred = 0.126;
  Qt = 1e-6;
  ErrerParameter[0] = 1e-5;
  ErrerParameter[1] = 1e-5;
  ErrerParameter[2] = 1e-5;
  ErrerParameter[3] = 1e-5;
  eml_rand_init();
  eml_rand_mcg16807_stateful_init();
  eml_rand_shr3cong_stateful_init();
}

/*
 * File trailer for GetSelfLocation_initialize.c
 *
 * [EOF]
 */
