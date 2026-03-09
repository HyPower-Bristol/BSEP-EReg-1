/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EReg_Tank_Drain_discrete.c
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

#include "EReg_Tank_Drain_discrete.h"
#include "EReg_Tank_Drain_discrete_private.h"
#include <math.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include <float.h>

/* Block signals (default storage) */
B_EReg_Tank_Drain_discrete_T EReg_Tank_Drain_discrete_B;

/* Continuous states */
X_EReg_Tank_Drain_discrete_T EReg_Tank_Drain_discrete_X;

/* Disabled State Vector */
XDis_EReg_Tank_Drain_discrete_T EReg_Tank_Drain_discrete_XDis;

/* Block states (default storage) */
DW_EReg_Tank_Drain_discrete_T EReg_Tank_Drain_discrete_DW;

/* Real-time model */
static RT_MODEL_EReg_Tank_Drain_disc_T EReg_Tank_Drain_discrete_M_;
RT_MODEL_EReg_Tank_Drain_disc_T *const EReg_Tank_Drain_discrete_M =
  &EReg_Tank_Drain_discrete_M_;
real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 5;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  EReg_Tank_Drain_discrete_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  EReg_Tank_Drain_discrete_step();
  EReg_Tank_Drain_discrete_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  EReg_Tank_Drain_discrete_step();
  EReg_Tank_Drain_discrete_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

real_T rt_remd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (rtIsInf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != trunc(u1))) {
    real_T q;
    q = fabs(u0 / u1);
    if (!(fabs(q - floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = fmod(u0, u1);
    }
  } else {
    y = fmod(u0, u1);
  }

  return y;
}

/* Model step function */
void EReg_Tank_Drain_discrete_step(void)
{
  /* local block i/o variables */
  real_T rtb_IProdOut;
  real_T rtb_NProdOut;
  real_T rtb_FilterCoefficient;
  real_T rtb_IntegralGain;
  if (rtmIsMajorTimeStep(EReg_Tank_Drain_discrete_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&EReg_Tank_Drain_discrete_M->solverInfo,
                          ((EReg_Tank_Drain_discrete_M->Timing.clockTick0+1)*
      EReg_Tank_Drain_discrete_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(EReg_Tank_Drain_discrete_M)) {
    EReg_Tank_Drain_discrete_M->Timing.t[0] = rtsiGetT
      (&EReg_Tank_Drain_discrete_M->solverInfo);
  }

  {
    real_T P_3_bar;
    real_T Q_N;
    real_T Switch;
    real_T b_delta_P;
    real_T b_delta_P_pa;
    real_T c_Q_N;
    real_T delta_P_pa;
    real_T rtb_AddConstant;
    real_T rtb_P_1;
    real_T rtb_P_2;
    real_T rtb_P_tankbar;
    int32_T i;
    boolean_T exitg1;
    boolean_T tmp;

    /* MATLAB Function: '<S6>/Gas Volume Properties' incorporates:
     *  Integrator: '<S6>/Integrator'
     *  Integrator: '<S6>/Integrator1'
     *  Integrator: '<S6>/Integrator2'
     */
    rtb_P_tankbar = EReg_Tank_Drain_discrete_X.Integrator_CSTATE / 0.0068;
    Q_N = rt_powd_snf(rtb_P_tankbar / 337.83783783783781, 0.39999999999999991) *
      300.0;
    rtb_P_1 = rtb_P_tankbar * 296.0 * Q_N;
    rtb_P_2 = EReg_Tank_Drain_discrete_X.Integrator1_CSTATE / (0.025 -
      EReg_Tank_Drain_discrete_X.Integrator2_CSTATE / 1000.0) * 296.0 * Q_N;

    /* Bias: '<Root>/Add Constant' incorporates:
     *  Gain: '<Root>/Gain1'
     *  Gain: '<Root>/Gain4'
     *  Gain: '<Root>/Gain5'
     *  Sum: '<Root>/Subtract2'
     */
    rtb_AddConstant = (EReg_Tank_Drain_discrete_ConstB.P_HPbar - 1.0E-5 *
                       rtb_P_1) * 0.0033333333333333335 * 5.0 + 1.0;

    /* Gain: '<Root>/Gain' */
    rtb_P_tankbar = 1.0E-5 * rtb_P_2;

    /* Clock: '<S4>/Clock' incorporates:
     *  Clock: '<S2>/Clock'
     *  Clock: '<S3>/Clock'
     *  Clock: '<S61>/Clock'
     */
    delta_P_pa = EReg_Tank_Drain_discrete_M->Timing.t[0];

    /* Switch: '<Root>/Switch' incorporates:
     *  Clock: '<S4>/Clock'
     *  Constant: '<S4>/Constant'
     *  Lookup_n-D: '<S4>/Look-Up Table1'
     *  Math: '<S4>/Math Function'
     *  S-Function (sfun_tstart): '<S4>/startTime'
     *  Sum: '<S4>/Sum'
     */
    if (look1_binlxpw(rt_remd_snf(delta_P_pa - (0.0), 10.0),
                      EReg_Tank_Drain_discrete_ConstP.pooled4,
                      EReg_Tank_Drain_discrete_ConstP.pooled8, 9999U) != 0.0) {
      /* Switch: '<Root>/Switch' incorporates:
       *  Constant: '<S2>/Constant'
       *  Lookup_n-D: '<S2>/Look-Up Table1'
       *  Math: '<S2>/Math Function'
       *  S-Function (sfun_tstart): '<S2>/startTime'
       *  Sum: '<Root>/Add'
       *  Sum: '<S2>/Sum'
       */
      Switch = look1_binlxpw(rt_remd_snf(delta_P_pa - (0.0), 10.0),
        EReg_Tank_Drain_discrete_ConstP.pooled4,
        EReg_Tank_Drain_discrete_ConstP.LookUpTable1_tableData, 9999U) -
        rtb_P_tankbar;
    } else {
      /* Switch: '<Root>/Switch' incorporates:
       *  Constant: '<Root>/Constant6'
       */
      Switch = 0.0;
    }

    /* End of Switch: '<Root>/Switch' */
    tmp = rtmIsMajorTimeStep(EReg_Tank_Drain_discrete_M);
    if (tmp) {
      /* Product: '<S38>/IProd Out' incorporates:
       *  Constant: '<Root>/Constant1'
       *  Product: '<Root>/Product1'
       */
      rtb_IProdOut = 17.0 * rtb_AddConstant * Switch;

      /* Product: '<S44>/NProd Out' incorporates:
       *  Constant: '<Root>/Constant4'
       *  Constant: '<Root>/Constant5'
       *  DiscreteIntegrator: '<S36>/Filter'
       *  Product: '<S34>/DProd Out'
       *  Sum: '<S36>/SumD'
       */
      rtb_NProdOut = (Switch * 8.0 - EReg_Tank_Drain_discrete_DW.Filter_DSTATE) *
        100.0;

      /* Sum: '<S50>/Sum' incorporates:
       *  Constant: '<Root>/Constant2'
       *  DiscreteIntegrator: '<S41>/Integrator'
       *  Product: '<Root>/Product'
       *  Product: '<S46>/PProd Out'
       */
      EReg_Tank_Drain_discrete_B.Sum = (16.0 * rtb_AddConstant * Switch +
        EReg_Tank_Drain_discrete_DW.Integrator_DSTATE) + rtb_NProdOut;
    }

    /* Switch: '<S5>/Switch' incorporates:
     *  Constant: '<S61>/Constant'
     *  Lookup_n-D: '<S61>/Look-Up Table1'
     *  Math: '<S61>/Math Function'
     *  S-Function (sfun_tstart): '<S61>/startTime'
     *  Sum: '<S61>/Sum'
     */
    if (look1_binlxpw(rt_remd_snf(delta_P_pa - (0.0), 10.0),
                      EReg_Tank_Drain_discrete_ConstP.pooled4,
                      EReg_Tank_Drain_discrete_ConstP.LookUpTable1_tableData_d,
                      9999U) != 0.0) {
      /* Switch: '<S5>/Switch' incorporates:
       *  Sum: '<Root>/Sum'
       */
      EReg_Tank_Drain_discrete_B.Switch_n = EReg_Tank_Drain_discrete_B.Sum;
    } else {
      /* Switch: '<S5>/Switch' incorporates:
       *  Constant: '<S5>/Constant6'
       */
      EReg_Tank_Drain_discrete_B.Switch_n = 0.0;
    }

    /* End of Switch: '<S5>/Switch' */

    /* MATLAB Function: '<S5>/ Valve Gear Conversion' incorporates:
     *  Integrator: '<S62>/Integrator'
     */
    rtb_AddConstant = EReg_Tank_Drain_discrete_X.Integrator_CSTATE_k / 1.7;

    /* Backlash: '<S5>/Backlash' */
    if (rtb_AddConstant < EReg_Tank_Drain_discrete_DW.PrevY - 0.25) {
      /* Backlash: '<S5>/Backlash' */
      EReg_Tank_Drain_discrete_B.Backlash = rtb_AddConstant + 0.25;
    } else if (rtb_AddConstant <= EReg_Tank_Drain_discrete_DW.PrevY + 0.25) {
      /* Backlash: '<S5>/Backlash' */
      EReg_Tank_Drain_discrete_B.Backlash = EReg_Tank_Drain_discrete_DW.PrevY;
    } else {
      /* Backlash: '<S5>/Backlash' */
      EReg_Tank_Drain_discrete_B.Backlash = rtb_AddConstant - 0.25;
    }

    /* End of Backlash: '<S5>/Backlash' */

    /* MATLAB Function: '<S62>/Signal normalisation' incorporates:
     *  VariableTransportDelay: '<S62>/Time Delay'
     */
    if (EReg_Tank_Drain_discrete_B.Switch_n < 0.0) {
      rtb_AddConstant = 0.0;
    } else if (EReg_Tank_Drain_discrete_B.Switch_n > 180.0) {
      rtb_AddConstant = 180.0;
    } else {
      rtb_AddConstant = EReg_Tank_Drain_discrete_B.Switch_n;
    }

    /* Sum: '<S62>/Subtract' incorporates:
     *  MATLAB Function: '<S62>/Signal normalisation'
     */
    rtb_AddConstant -= EReg_Tank_Drain_discrete_B.Backlash;
    if (tmp) {
      /* Gain: '<S102>/Filter Coefficient' incorporates:
       *  DiscreteIntegrator: '<S94>/Filter'
       *  Gain: '<S92>/Derivative Gain'
       *  Sum: '<S94>/SumD'
       */
      rtb_FilterCoefficient = (0.0 * rtb_AddConstant -
        EReg_Tank_Drain_discrete_DW.Filter_DSTATE_o) * 100.0;

      /* Sum: '<S109>/Sum' incorporates:
       *  DiscreteIntegrator: '<S99>/Integrator'
       *  Gain: '<S104>/Proportional Gain'
       */
      EReg_Tank_Drain_discrete_B.Switch2 = (3.0 * rtb_AddConstant +
        EReg_Tank_Drain_discrete_DW.Integrator_DSTATE_b) + rtb_FilterCoefficient;

      /* Switch: '<S107>/Switch2' incorporates:
       *  Constant: '<S62>/Servo max reverse speed [deg//s]'
       *  Constant: '<S62>/Servo max speed [deg//s]'
       *  RelationalOperator: '<S107>/LowerRelop1'
       *  RelationalOperator: '<S107>/UpperRelop'
       *  Switch: '<S107>/Switch'
       */
      if (EReg_Tank_Drain_discrete_B.Switch2 > 180.0) {
        /* Sum: '<S109>/Sum' incorporates:
         *  Switch: '<S107>/Switch2'
         */
        EReg_Tank_Drain_discrete_B.Switch2 = 180.0;
      } else if (EReg_Tank_Drain_discrete_B.Switch2 < -180.0) {
        /* Sum: '<S109>/Sum' incorporates:
         *  Constant: '<S62>/Servo max reverse speed [deg//s]'
         *  Switch: '<S107>/Switch'
         *  Switch: '<S107>/Switch2'
         */
        EReg_Tank_Drain_discrete_B.Switch2 = -180.0;
      }

      /* End of Switch: '<S107>/Switch2' */

      /* Gain: '<S96>/Integral Gain' */
      rtb_IntegralGain = 2.0 * rtb_AddConstant;
    }

    /* MATLAB Function: '<S5>/ Valve area' */
    if (EReg_Tank_Drain_discrete_B.Backlash - 5.0 < 0.0) {
      rtb_AddConstant = 0.0;
    } else {
      rtb_AddConstant = ((rt_powd_snf(EReg_Tank_Drain_discrete_B.Backlash - 5.0,
        3.0) * 4.9999999999999996E-6 - (EReg_Tank_Drain_discrete_B.Backlash -
        5.0) * (EReg_Tank_Drain_discrete_B.Backlash - 5.0) * 0.0006) +
                         (EReg_Tank_Drain_discrete_B.Backlash - 5.0) * 0.0278) *
        1.1;
    }

    /* End of MATLAB Function: '<S5>/ Valve area' */

    /* Lookup_n-D: '<S3>/Look-Up Table1' incorporates:
     *  Constant: '<S3>/Constant'
     *  Math: '<S3>/Math Function'
     *  S-Function (sfun_tstart): '<S3>/startTime'
     *  Sum: '<S3>/Sum'
     */
    Switch = look1_binlxpw(rt_remd_snf(delta_P_pa - (0.0), 10.0),
      EReg_Tank_Drain_discrete_ConstP.pooled4,
      EReg_Tank_Drain_discrete_ConstP.pooled8, 9999U);

    /* MATLAB Function: '<S6>/Component Flow Solver' incorporates:
     *  MATLAB Function: '<S6>/Gas Volume Properties'
     */
    rtb_P_1 *= 1.0E-5;
    rtb_P_2 *= 1.0E-5;
    P_3_bar = 1.0;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= 49)) {
      b_delta_P = rtb_P_2 - P_3_bar;
      delta_P_pa = P_3_bar * 100000.0 - 101325.0;
      b_delta_P = Switch * b_delta_P / sqrt(fabs(b_delta_P) + 1.0E-9) * 1000.0 *
        0.00027777777777777778 - 0.083999999999999977 * delta_P_pa / sqrt(fabs
        (2000.0 * delta_P_pa) + 1.0E-9);
      if (fabs(b_delta_P) < 1.0E-7) {
        exitg1 = true;
      } else {
        delta_P_pa = rtb_P_2 - (P_3_bar + 1.0E-6);
        b_delta_P_pa = (P_3_bar + 1.0E-6) * 100000.0 - 101325.0;
        delta_P_pa = ((Switch * delta_P_pa / sqrt(fabs(delta_P_pa) + 1.0E-9) *
                       1000.0 * 0.00027777777777777778 - 0.083999999999999977 *
                       b_delta_P_pa / sqrt(fabs(2000.0 * b_delta_P_pa) + 1.0E-9))
                      - b_delta_P) / 1.0E-6;
        if (fabs(delta_P_pa) < 1.0E-10) {
          exitg1 = true;
        } else {
          b_delta_P = P_3_bar - b_delta_P / delta_P_pa;
          if (fabs(b_delta_P - P_3_bar) < 1.0E-7) {
            P_3_bar = b_delta_P;
            exitg1 = true;
          } else {
            P_3_bar = b_delta_P;
            i++;
          }
        }
      }
    }

    b_delta_P = 1.0;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i <= 49)) {
      delta_P_pa = 0.528 * rtb_P_1;
      if (b_delta_P >= delta_P_pa) {
        b_delta_P_pa = (rtb_P_1 - b_delta_P) * b_delta_P / (1.2532095522210844 *
          Q_N);
        b_delta_P_pa = rtb_AddConstant * 514.0 * b_delta_P_pa / sqrt(fabs
          (b_delta_P_pa) + 1.0E-9);
      } else {
        b_delta_P_pa = rtb_AddConstant * 257.0 * rtb_P_1 / sqrt
          (1.2532095522210844 * Q_N + 1.0E-9);
      }

      if (rtb_P_2 >= 0.528 * b_delta_P) {
        c_Q_N = (b_delta_P - rtb_P_2) * rtb_P_2 / (1.2532095522210844 * Q_N);
        c_Q_N = 257.0 * c_Q_N / sqrt(fabs(c_Q_N) + 1.0E-9);
      } else {
        c_Q_N = 128.5 * b_delta_P / sqrt(1.2532095522210844 * Q_N + 1.0E-9);
      }

      b_delta_P_pa = b_delta_P_pa * 1.2532095522210844 * 0.00027777777777777778
        - c_Q_N * 1.2532095522210844 * 0.00027777777777777778;
      if (fabs(b_delta_P_pa) < 1.0E-7) {
        exitg1 = true;
      } else {
        if (b_delta_P + 1.0E-6 >= delta_P_pa) {
          delta_P_pa = (rtb_P_1 - (b_delta_P + 1.0E-6)) * (b_delta_P + 1.0E-6) /
            (1.2532095522210844 * Q_N);
          delta_P_pa = rtb_AddConstant * 514.0 * delta_P_pa / sqrt(fabs
            (delta_P_pa) + 1.0E-9);
        } else {
          delta_P_pa = rtb_AddConstant * 257.0 * rtb_P_1 / sqrt
            (1.2532095522210844 * Q_N + 1.0E-9);
        }

        if (rtb_P_2 >= (b_delta_P + 1.0E-6) * 0.528) {
          c_Q_N = ((b_delta_P + 1.0E-6) - rtb_P_2) * rtb_P_2 /
            (1.2532095522210844 * Q_N);
          c_Q_N = 257.0 * c_Q_N / sqrt(fabs(c_Q_N) + 1.0E-9);
        } else {
          c_Q_N = (b_delta_P + 1.0E-6) * 128.5 / sqrt(1.2532095522210844 * Q_N +
            1.0E-9);
        }

        delta_P_pa = ((delta_P_pa * 1.2532095522210844 * 0.00027777777777777778
                       - c_Q_N * 1.2532095522210844 * 0.00027777777777777778) -
                      b_delta_P_pa) / 1.0E-6;
        if (fabs(delta_P_pa) < 1.0E-10) {
          exitg1 = true;
        } else {
          delta_P_pa = b_delta_P - b_delta_P_pa / delta_P_pa;
          if (fabs(delta_P_pa - b_delta_P) < 1.0E-7) {
            b_delta_P = delta_P_pa;
            exitg1 = true;
          } else {
            b_delta_P = delta_P_pa;
            i++;
          }
        }
      }
    }

    rtb_P_2 -= P_3_bar;
    Switch = Switch * rtb_P_2 / sqrt(fabs(rtb_P_2) + 1.0E-9) * 1000.0 *
      0.00027777777777777778;
    if (b_delta_P >= 0.528 * rtb_P_1) {
      Q_N = (rtb_P_1 - b_delta_P) * b_delta_P / (1.2532095522210844 * Q_N);
      Q_N = rtb_AddConstant * 514.0 * Q_N / sqrt(fabs(Q_N) + 1.0E-9);
    } else {
      Q_N = rtb_AddConstant * 257.0 * rtb_P_1 / sqrt(1.2532095522210844 * Q_N +
        1.0E-9);
    }

    EReg_Tank_Drain_discrete_B.m_dot_N2 = Q_N * 1.2532095522210844 *
      0.00027777777777777778;

    /* End of MATLAB Function: '<S6>/Component Flow Solver' */

    /* Gain: '<S6>/Gain' */
    EReg_Tank_Drain_discrete_B.m_dot_1 = -EReg_Tank_Drain_discrete_B.m_dot_N2;

    /* Saturate: '<S6>/Saturation' */
    if (Switch <= 0.0) {
      /* Gain: '<S6>/Gain1' */
      EReg_Tank_Drain_discrete_B.m_dot_3 = -0.0;
    } else {
      /* Gain: '<S6>/Gain1' */
      EReg_Tank_Drain_discrete_B.m_dot_3 = -Switch;
    }

    /* End of Saturate: '<S6>/Saturation' */

    /* Abs: '<Root>/Abs' incorporates:
     *  Sum: '<Root>/Subtract'
     */
    EReg_Tank_Drain_discrete_B.Abs = fabs(rtb_P_tankbar);
  }

  if (rtmIsMajorTimeStep(EReg_Tank_Drain_discrete_M)) {
    boolean_T bufferFull;
    boolean_T tmp;
    tmp = rtmIsMajorTimeStep(EReg_Tank_Drain_discrete_M);
    if (tmp) {
      /* Update for DiscreteIntegrator: '<S36>/Filter' */
      EReg_Tank_Drain_discrete_DW.Filter_DSTATE += 0.002 * rtb_NProdOut;

      /* Update for DiscreteIntegrator: '<S41>/Integrator' */
      EReg_Tank_Drain_discrete_DW.Integrator_DSTATE += 0.002 * rtb_IProdOut;
      if (EReg_Tank_Drain_discrete_DW.Integrator_DSTATE > 20.0) {
        EReg_Tank_Drain_discrete_DW.Integrator_DSTATE = 20.0;
      } else if (EReg_Tank_Drain_discrete_DW.Integrator_DSTATE < -20.0) {
        EReg_Tank_Drain_discrete_DW.Integrator_DSTATE = -20.0;
      }

      /* End of Update for DiscreteIntegrator: '<S41>/Integrator' */
    }

    /* Update for VariableTransportDelay: '<S62>/Time Delay' */
    bufferFull = false;
    if (EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[1] <
        EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[3] - 1) {
      EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[1]++;
    } else {
      EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[1] = 0;
    }

    if (EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[0] ==
        EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[1]) {
      bufferFull = true;
      if (EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[0] <
          EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[3] - 1) {
        EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[0]++;
      } else {
        EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[0] = 0;
      }
    }

    ((real_T *)EReg_Tank_Drain_discrete_DW.TimeDelay_PWORK[0])
      [EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[1]] =
      EReg_Tank_Drain_discrete_B.Switch_n;
    ((real_T *)EReg_Tank_Drain_discrete_DW.TimeDelay_PWORK[0])
      [EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[1] +
      EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[3]] =
      EReg_Tank_Drain_discrete_M->Timing.t[0];
    if (bufferFull) {
      rtsiSetBlockStateForSolverChangedAtMajorStep
        (&EReg_Tank_Drain_discrete_M->solverInfo, true);
      rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
        (&EReg_Tank_Drain_discrete_M->solverInfo, true);
    }

    /* End of Update for VariableTransportDelay: '<S62>/Time Delay' */

    /* Update for Backlash: '<S5>/Backlash' */
    EReg_Tank_Drain_discrete_DW.PrevY = EReg_Tank_Drain_discrete_B.Backlash;
    if (tmp) {
      /* Update for DiscreteIntegrator: '<S94>/Filter' */
      EReg_Tank_Drain_discrete_DW.Filter_DSTATE_o += 0.002 *
        rtb_FilterCoefficient;

      /* Update for DiscreteIntegrator: '<S99>/Integrator' */
      EReg_Tank_Drain_discrete_DW.Integrator_DSTATE_b += 0.002 *
        rtb_IntegralGain;
    }

    /* ContTimeOutputInconsistentWithStateAtMajorOutputFlag is set, need to run a minor output */
    if (rtmIsMajorTimeStep(EReg_Tank_Drain_discrete_M)) {
      if (rtsiGetContTimeOutputInconsistentWithStateAtMajorStep
          (&EReg_Tank_Drain_discrete_M->solverInfo)) {
        rtsiSetSimTimeStep(&EReg_Tank_Drain_discrete_M->solverInfo,
                           MINOR_TIME_STEP);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&EReg_Tank_Drain_discrete_M->solverInfo, false);
        EReg_Tank_Drain_discrete_step();
        rtsiSetSimTimeStep(&EReg_Tank_Drain_discrete_M->solverInfo,
                           MAJOR_TIME_STEP);
      }
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(EReg_Tank_Drain_discrete_M)) {
    rt_ertODEUpdateContinuousStates(&EReg_Tank_Drain_discrete_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++EReg_Tank_Drain_discrete_M->Timing.clockTick0;
    EReg_Tank_Drain_discrete_M->Timing.t[0] = rtsiGetSolverStopTime
      (&EReg_Tank_Drain_discrete_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.002s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.002, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      EReg_Tank_Drain_discrete_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void EReg_Tank_Drain_discrete_derivatives(void)
{
  XDot_EReg_Tank_Drain_discrete_T *_rtXdot;
  _rtXdot = ((XDot_EReg_Tank_Drain_discrete_T *)
             EReg_Tank_Drain_discrete_M->derivs);

  /* Derivatives for Integrator: '<S6>/Integrator' */
  _rtXdot->Integrator_CSTATE = EReg_Tank_Drain_discrete_B.m_dot_1;

  /* Derivatives for Integrator: '<S6>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = EReg_Tank_Drain_discrete_B.m_dot_N2;

  /* Derivatives for Integrator: '<S6>/Integrator2' */
  _rtXdot->Integrator2_CSTATE = EReg_Tank_Drain_discrete_B.m_dot_3;

  /* Derivatives for Integrator: '<S62>/Integrator' */
  _rtXdot->Integrator_CSTATE_k = EReg_Tank_Drain_discrete_B.Switch2;

  /* Derivatives for Integrator: '<Root>/Integrator' */
  _rtXdot->Integrator_CSTATE_h = EReg_Tank_Drain_discrete_B.Abs;
}

/* Model initialize function */
void EReg_Tank_Drain_discrete_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&EReg_Tank_Drain_discrete_M->solverInfo,
                          &EReg_Tank_Drain_discrete_M->Timing.simTimeStep);
    rtsiSetTPtr(&EReg_Tank_Drain_discrete_M->solverInfo, &rtmGetTPtr
                (EReg_Tank_Drain_discrete_M));
    rtsiSetStepSizePtr(&EReg_Tank_Drain_discrete_M->solverInfo,
                       &EReg_Tank_Drain_discrete_M->Timing.stepSize0);
    rtsiSetdXPtr(&EReg_Tank_Drain_discrete_M->solverInfo,
                 &EReg_Tank_Drain_discrete_M->derivs);
    rtsiSetContStatesPtr(&EReg_Tank_Drain_discrete_M->solverInfo, (real_T **)
                         &EReg_Tank_Drain_discrete_M->contStates);
    rtsiSetNumContStatesPtr(&EReg_Tank_Drain_discrete_M->solverInfo,
      &EReg_Tank_Drain_discrete_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&EReg_Tank_Drain_discrete_M->solverInfo,
      &EReg_Tank_Drain_discrete_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&EReg_Tank_Drain_discrete_M->solverInfo,
      &EReg_Tank_Drain_discrete_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&EReg_Tank_Drain_discrete_M->solverInfo,
      &EReg_Tank_Drain_discrete_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&EReg_Tank_Drain_discrete_M->solverInfo,
      (boolean_T**) &EReg_Tank_Drain_discrete_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&EReg_Tank_Drain_discrete_M->solverInfo,
                          (&rtmGetErrorStatus(EReg_Tank_Drain_discrete_M)));
    rtsiSetRTModelPtr(&EReg_Tank_Drain_discrete_M->solverInfo,
                      EReg_Tank_Drain_discrete_M);
  }

  rtsiSetSimTimeStep(&EReg_Tank_Drain_discrete_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&EReg_Tank_Drain_discrete_M->solverInfo,
    false);
  rtsiSetIsContModeFrozen(&EReg_Tank_Drain_discrete_M->solverInfo, false);
  EReg_Tank_Drain_discrete_M->intgData.y = EReg_Tank_Drain_discrete_M->odeY;
  EReg_Tank_Drain_discrete_M->intgData.f[0] = EReg_Tank_Drain_discrete_M->odeF[0];
  EReg_Tank_Drain_discrete_M->intgData.f[1] = EReg_Tank_Drain_discrete_M->odeF[1];
  EReg_Tank_Drain_discrete_M->intgData.f[2] = EReg_Tank_Drain_discrete_M->odeF[2];
  EReg_Tank_Drain_discrete_M->contStates = ((X_EReg_Tank_Drain_discrete_T *)
    &EReg_Tank_Drain_discrete_X);
  EReg_Tank_Drain_discrete_M->contStateDisabled =
    ((XDis_EReg_Tank_Drain_discrete_T *) &EReg_Tank_Drain_discrete_XDis);
  EReg_Tank_Drain_discrete_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&EReg_Tank_Drain_discrete_M->solverInfo, (void *)
                    &EReg_Tank_Drain_discrete_M->intgData);
  rtsiSetSolverName(&EReg_Tank_Drain_discrete_M->solverInfo,"ode3");
  rtmSetTPtr(EReg_Tank_Drain_discrete_M,
             &EReg_Tank_Drain_discrete_M->Timing.tArray[0]);
  EReg_Tank_Drain_discrete_M->Timing.stepSize0 = 0.002;

  {
    int_T j;

    /* Start for VariableTransportDelay: '<S62>/Time Delay' */
    EReg_Tank_Drain_discrete_DW.TimeDelay_RWORK[0] = 0.0;
    EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[3] = 6000;
    EReg_Tank_Drain_discrete_DW.TimeDelay_PWORK[0] = (void *)
      &EReg_Tank_Drain_discrete_DW.TimeDelay_RWORK[1];
    for (j = 0; j < 6000; j++) {
      EReg_Tank_Drain_discrete_DW.TimeDelay_RWORK[j + 1] = 0.0;
      EReg_Tank_Drain_discrete_DW.TimeDelay_RWORK[j + 6001] =
        EReg_Tank_Drain_discrete_M->Timing.t[0];
    }

    /* End of Start for VariableTransportDelay: '<S62>/Time Delay' */

    /* InitializeConditions for Integrator: '<S6>/Integrator' */
    EReg_Tank_Drain_discrete_X.Integrator_CSTATE = 2.2972972972972969;

    /* InitializeConditions for Integrator: '<S6>/Integrator1' */
    EReg_Tank_Drain_discrete_X.Integrator1_CSTATE = 0.2815315315315316;

    /* InitializeConditions for Integrator: '<S6>/Integrator2' */
    EReg_Tank_Drain_discrete_X.Integrator2_CSTATE = 20.0;

    /* InitializeConditions for VariableTransportDelay: '<S62>/Time Delay' */
    EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[0] = 0;
    EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[1] = 0;
    EReg_Tank_Drain_discrete_DW.TimeDelay_IWORK[2] = 0;
    ((real_T *)EReg_Tank_Drain_discrete_DW.TimeDelay_PWORK[0])[0] = 0.0;
    ((real_T *)EReg_Tank_Drain_discrete_DW.TimeDelay_PWORK[0])[6000] =
      EReg_Tank_Drain_discrete_M->Timing.t[0];

    /* InitializeConditions for Integrator: '<S62>/Integrator' */
    EReg_Tank_Drain_discrete_X.Integrator_CSTATE_k = 0.0;

    /* InitializeConditions for Backlash: '<S5>/Backlash' */
    EReg_Tank_Drain_discrete_DW.PrevY = 0.5;

    /* InitializeConditions for Integrator: '<Root>/Integrator' */
    EReg_Tank_Drain_discrete_X.Integrator_CSTATE_h = 0.0;
  }
}

/* Model terminate function */
void EReg_Tank_Drain_discrete_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
