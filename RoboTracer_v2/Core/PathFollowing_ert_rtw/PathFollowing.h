/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PathFollowing.h
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

#ifndef RTW_HEADER_PathFollowing_h_
#define RTW_HEADER_PathFollowing_h_

#include <math.h>
#ifndef PathFollowing_COMMON_INCLUDES_
# define PathFollowing_COMMON_INCLUDES_
#include "rtwtypes.h"

#endif                                 /* PathFollowing_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Add4;                         /* '<S2>/Add4' */
  real_T Add5;                         /* '<S2>/Add5' */
  real_T Add3;                         /* '<S2>/Add3' */
  real_T DiscreteTransferFcn_states;   /* '<S7>/Discrete Transfer Fcn' */
  real_T DiscreteTransferFcn_states_g; /* '<S8>/Discrete Transfer Fcn' */
  real_T DiscreteTransferFcn_states_d; /* '<S6>/Discrete Transfer Fcn' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T X_r;                          /* '<Root>/x_ref' */
  real_T Y_r;                          /* '<Root>/y_ref' */
  real_T Th_r;                         /* '<Root>/th_ref ' */
  real_T X_c;                          /* '<Root>/x_cur ' */
  real_T Y_c;                          /* '<Root>/y_cur' */
  real_T Th_c;                         /* '<Root>/th_cur' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T v_tar;                        /* '<Root>/v_tar' */
  real_T _tar;                         /* '<Root>/��_tar' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void PathFollowing_initialize(void);
extern void PathFollowing_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('simlation/PathFollowing')    - opens subsystem simlation/PathFollowing
 * hilite_system('simlation/PathFollowing/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'simlation'
 * '<S1>'   : 'simlation/PathFollowing'
 * '<S2>'   : 'simlation/PathFollowing/CalcError1'
 * '<S3>'   : 'simlation/PathFollowing/ClacInputRef '
 * '<S4>'   : 'simlation/PathFollowing/ClacTarget_V��1'
 * '<S5>'   : 'simlation/PathFollowing/Limitter'
 * '<S6>'   : 'simlation/PathFollowing/ClacInputRef /Derivative1'
 * '<S7>'   : 'simlation/PathFollowing/ClacInputRef /Derivative5'
 * '<S8>'   : 'simlation/PathFollowing/ClacInputRef /Derivative6'
 */
#endif                                 /* RTW_HEADER_PathFollowing_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
