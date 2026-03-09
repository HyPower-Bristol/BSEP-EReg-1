/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EReg_Tank_Drain_discrete_private.h
 *
 * Code generated for Simulink model 'EReg_Tank_Drain_discrete'.
 *
 * Model version                  : 7.13
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Mon Mar  9 13:55:48 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef EReg_Tank_Drain_discrete_private_h_
#define EReg_Tank_Drain_discrete_private_h_
#include "rtwtypes.h"
#include "EReg_Tank_Drain_discrete_types.h"
#include "EReg_Tank_Drain_discrete.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern real_T rt_remd_snf(real_T u0, real_T u1);
extern real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);

/* private model entry point functions */
extern void EReg_Tank_Drain_discrete_derivatives(void);

#endif                                 /* EReg_Tank_Drain_discrete_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
