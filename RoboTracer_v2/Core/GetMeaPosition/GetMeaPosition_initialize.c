/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: GetMeaPosition_initialize.c
 *
 * MATLAB Coder version            : 4.2
 * C/C++ source code generated on  : 24-Mar-2020 17:53:43
 */

/* Include Files */
#include "GetMeaPosition.h"
#include "GetMeaPosition_initialize.h"
#include "eml_rand_shr3cong_stateful.h"
#include "eml_rand_mcg16807_stateful.h"
#include "eml_rand.h"
#include "eml_randn.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "GetMeaPosition_data.h"

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void GetMeaPosition_initialize(void)
{
  state_not_empty_init();
  method_not_empty_init();
  ErrerParameter[0] = 10;
  ErrerParameter[1] = 10;
  ErrerParameter[2] = 10;
  ErrerParameter[3] = 10;
  eml_rand_init();
  eml_rand_mcg16807_stateful_init();
  eml_rand_shr3cong_stateful_init();
}

/*
 * File trailer for GetMeaPosition_initialize.c
 *
 * [EOF]
 */
