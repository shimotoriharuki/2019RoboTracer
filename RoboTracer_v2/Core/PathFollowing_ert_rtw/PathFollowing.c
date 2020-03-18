/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PathFollowing.c
 *
 * Code generated for Simulink model 'PathFollowing'.
 *
 * Model version                  : 1.4
 * Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
 * C/C++ source code generated on : Tue Mar 17 16:39:07 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->ST10/Super10
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 *    3. Traceability
 * Validation result: Not run
 */

#include "PathFollowing.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void CalcError1(void);

/* Output and update for atomic system: '<S1>/CalcError1' */
static void CalcError1(void)
{
  real_T rtb_Add1;
  real_T rtb_Add2;
  real_T Add4_tmp;
  real_T Add4_tmp_0;

  /* Sum: '<S2>/Add1' incorporates:
   *  Inport: '<Root>/y_cur'
   *  Inport: '<Root>/y_ref'
   */
  rtb_Add1 = rtU.Y_r - rtU.Y_c;

  /* Sum: '<S2>/Add2' incorporates:
   *  Inport: '<Root>/x_cur '
   *  Inport: '<Root>/x_ref'
   */
  rtb_Add2 = rtU.X_r - rtU.X_c;

  /* Trigonometry: '<S2>/Trigonometric Function1' incorporates:
   *  Inport: '<Root>/th_cur'
   *  Trigonometry: '<S2>/Trigonometric Function2'
   */
  Add4_tmp = sin(rtU.Th_c);

  /* Trigonometry: '<S2>/Trigonometric Function4' incorporates:
   *  Inport: '<Root>/th_cur'
   *  Trigonometry: '<S2>/Trigonometric Function3'
   */
  Add4_tmp_0 = cos(rtU.Th_c);

  /* Sum: '<S2>/Add4' incorporates:
   *  Product: '<S2>/Product'
   *  Product: '<S2>/Product1'
   *  Trigonometry: '<S2>/Trigonometric Function1'
   *  Trigonometry: '<S2>/Trigonometric Function4'
   */
  rtDW.Add4 = rtb_Add2 * Add4_tmp_0 + rtb_Add1 * Add4_tmp;

  /* Sum: '<S2>/Add5' incorporates:
   *  Gain: '<S2>/Gain4'
   *  Product: '<S2>/Product2'
   *  Product: '<S2>/Product3'
   */
  rtDW.Add5 = rtb_Add2 * -Add4_tmp + rtb_Add1 * Add4_tmp_0;

  /* Sum: '<S2>/Add3' incorporates:
   *  Inport: '<Root>/th_cur'
   *  Inport: '<Root>/th_ref '
   */
  rtDW.Add3 = rtU.Th_r - rtU.Th_c;
}

/* Model step function */
void PathFollowing_step(void)
{
  real_T rtb_Add5_e;
  real_T rtb_Square;
  real_T rtb_DiscreteTransferFcn;
  real_T DiscreteTransferFcn_tmp;
  real_T DiscreteTransferFcn_tmp_l;

  /* Outputs for Atomic SubSystem: '<Root>/PathFollowing' */
  /* Outputs for Atomic SubSystem: '<S1>/CalcError1' */
  CalcError1();

  /* End of Outputs for SubSystem: '<S1>/CalcError1' */

  /* Outputs for Atomic SubSystem: '<S1>/ClacInputRef ' */
  /* DiscreteTransferFcn: '<S7>/Discrete Transfer Fcn' incorporates:
   *  Inport: '<Root>/x_ref'
   */
  DiscreteTransferFcn_tmp = rtU.X_r - rtDW.DiscreteTransferFcn_states;
  rtb_Add5_e = 200.0 * DiscreteTransferFcn_tmp + -200.0 *
    rtDW.DiscreteTransferFcn_states;

  /* Math: '<S3>/Square' */
  rtb_Square = rtb_Add5_e * rtb_Add5_e;

  /* DiscreteTransferFcn: '<S8>/Discrete Transfer Fcn' incorporates:
   *  Inport: '<Root>/y_ref'
   */
  DiscreteTransferFcn_tmp_l = rtU.Y_r - rtDW.DiscreteTransferFcn_states_g;
  rtb_Add5_e = 200.0 * DiscreteTransferFcn_tmp_l + -200.0 *
    rtDW.DiscreteTransferFcn_states_g;

  /* Sqrt: '<S3>/Sqrt' incorporates:
   *  Math: '<S3>/Square1'
   *  Sum: '<S3>/Add5'
   */
  rtb_Square = sqrt(rtb_Add5_e * rtb_Add5_e + rtb_Square);

  /* DiscreteTransferFcn: '<S6>/Discrete Transfer Fcn' incorporates:
   *  Inport: '<Root>/th_ref '
   */
  rtb_Add5_e = rtU.Th_r - rtDW.DiscreteTransferFcn_states_d;
  rtb_DiscreteTransferFcn = 200.0 * rtb_Add5_e + -200.0 *
    rtDW.DiscreteTransferFcn_states_d;

  /* Update for DiscreteTransferFcn: '<S7>/Discrete Transfer Fcn' */
  rtDW.DiscreteTransferFcn_states = DiscreteTransferFcn_tmp;

  /* Update for DiscreteTransferFcn: '<S8>/Discrete Transfer Fcn' */
  rtDW.DiscreteTransferFcn_states_g = DiscreteTransferFcn_tmp_l;

  /* Update for DiscreteTransferFcn: '<S6>/Discrete Transfer Fcn' */
  rtDW.DiscreteTransferFcn_states_d = rtb_Add5_e;

  /* End of Outputs for SubSystem: '<S1>/ClacInputRef ' */

  /* Outputs for Atomic SubSystem: '<S1>/ClacTarget_VÉ÷1' */
  /* Outport: '<Root>/v_tar' incorporates:
   *  Gain: '<S4>/Gain'
   *  Product: '<S4>/Product1'
   *  Sum: '<S4>/Add5'
   *  Trigonometry: '<S4>/Trigonometric Function2'
   */
  rtY.v_tar = 50.0 * rtDW.Add4 + cos(rtDW.Add3) * rtb_Square;

  /* Outport: '<Root>/É÷_tar' incorporates:
   *  Gain: '<S4>/Gain1'
   *  Gain: '<S4>/Gain2'
   *  Product: '<S4>/Product2'
   *  Sum: '<S4>/Add1'
   *  Sum: '<S4>/Add2'
   *  Trigonometry: '<S4>/Trigonometric Function1'
   */
  rtY._tar = (0.04 * rtDW.Add5 + 0.4 * sin(rtDW.Add3)) * rtb_Square +
    rtb_DiscreteTransferFcn;

  /* End of Outputs for SubSystem: '<S1>/ClacTarget_VÉ÷1' */
  /* End of Outputs for SubSystem: '<Root>/PathFollowing' */
}

/* Model initialize function */
void PathFollowing_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
