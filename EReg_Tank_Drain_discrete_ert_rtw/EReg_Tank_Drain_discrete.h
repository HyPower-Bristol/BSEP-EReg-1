/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: EReg_Tank_Drain_discrete.h
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

#ifndef EReg_Tank_Drain_discrete_h_
#define EReg_Tank_Drain_discrete_h_
#ifndef EReg_Tank_Drain_discrete_COMMON_INCLUDES_
#define EReg_Tank_Drain_discrete_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                           /* EReg_Tank_Drain_discrete_COMMON_INCLUDES_ */

#include "EReg_Tank_Drain_discrete_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Sum;                          /* '<S50>/Sum' */
  real_T Switch_n;                     /* '<S5>/Switch' */
  real_T Backlash;                     /* '<S5>/Backlash' */
  real_T Switch2;                      /* '<S107>/Switch2' */
  real_T m_dot_1;                      /* '<S6>/Gain' */
  real_T m_dot_3;                      /* '<S6>/Gain1' */
  real_T Abs;                          /* '<Root>/Abs' */
  real_T m_dot_N2;                     /* '<S6>/Component Flow Solver' */
} B_EReg_Tank_Drain_discrete_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Filter_DSTATE;                /* '<S36>/Filter' */
  real_T Integrator_DSTATE;            /* '<S41>/Integrator' */
  real_T Filter_DSTATE_o;              /* '<S94>/Filter' */
  real_T Integrator_DSTATE_b;          /* '<S99>/Integrator' */
  real_T PrevY;                        /* '<S5>/Backlash' */
  real_T TimeDelay_RWORK[12001];       /* '<S62>/Time Delay' */
  void *TimeDelay_PWORK[2];            /* '<S62>/Time Delay' */
  int_T TimeDelay_IWORK[4];            /* '<S62>/Time Delay' */
} DW_EReg_Tank_Drain_discrete_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S6>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<S6>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<S6>/Integrator2' */
  real_T Integrator_CSTATE_k;          /* '<S62>/Integrator' */
  real_T Integrator_CSTATE_h;          /* '<Root>/Integrator' */
} X_EReg_Tank_Drain_discrete_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<S6>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<S6>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<S6>/Integrator2' */
  real_T Integrator_CSTATE_k;          /* '<S62>/Integrator' */
  real_T Integrator_CSTATE_h;          /* '<Root>/Integrator' */
} XDot_EReg_Tank_Drain_discrete_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<S6>/Integrator' */
  boolean_T Integrator1_CSTATE;        /* '<S6>/Integrator1' */
  boolean_T Integrator2_CSTATE;        /* '<S6>/Integrator2' */
  boolean_T Integrator_CSTATE_k;       /* '<S62>/Integrator' */
  boolean_T Integrator_CSTATE_h;       /* '<Root>/Integrator' */
} XDis_EReg_Tank_Drain_discrete_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T P_HPbar;                /* '<Root>/Gain3' */
} ConstB_EReg_Tank_Drain_discre_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: rep_seq_y
   * Referenced by: '<S2>/Look-Up Table1'
   */
  real_T LookUpTable1_tableData[10000];

  /* Pooled Parameter (Expression: rep_seq_t - min(rep_seq_t))
   * Referenced by:
   *   '<S2>/Look-Up Table1'
   *   '<S3>/Look-Up Table1'
   *   '<S4>/Look-Up Table1'
   *   '<S61>/Look-Up Table1'
   */
  real_T pooled4[10000];

  /* Pooled Parameter (Expression: rep_seq_y)
   * Referenced by:
   *   '<S3>/Look-Up Table1'
   *   '<S4>/Look-Up Table1'
   */
  real_T pooled8[10000];

  /* Expression: rep_seq_y
   * Referenced by: '<S61>/Look-Up Table1'
   */
  real_T LookUpTable1_tableData_d[10000];
} ConstP_EReg_Tank_Drain_discre_T;

/* Real-time Model Data Structure */
struct tag_RTM_EReg_Tank_Drain_discr_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_EReg_Tank_Drain_discrete_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_EReg_Tank_Drain_discrete_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[5];
  real_T odeF[3][5];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_EReg_Tank_Drain_discrete_T EReg_Tank_Drain_discrete_B;

/* Continuous states (default storage) */
extern X_EReg_Tank_Drain_discrete_T EReg_Tank_Drain_discrete_X;

/* Disabled states (default storage) */
extern XDis_EReg_Tank_Drain_discrete_T EReg_Tank_Drain_discrete_XDis;

/* Block states (default storage) */
extern DW_EReg_Tank_Drain_discrete_T EReg_Tank_Drain_discrete_DW;
extern const ConstB_EReg_Tank_Drain_discre_T EReg_Tank_Drain_discrete_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_EReg_Tank_Drain_discre_T EReg_Tank_Drain_discrete_ConstP;

/* Model entry point functions */
extern void EReg_Tank_Drain_discrete_initialize(void);
extern void EReg_Tank_Drain_discrete_step(void);
extern void EReg_Tank_Drain_discrete_terminate(void);

/* Real-time Model object */
extern RT_MODEL_EReg_Tank_Drain_disc_T *const EReg_Tank_Drain_discrete_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<Root>/Scope10' : Unused code path elimination
 * Block '<Root>/Scope11' : Unused code path elimination
 * Block '<Root>/Scope12' : Unused code path elimination
 * Block '<Root>/Scope13' : Unused code path elimination
 * Block '<Root>/Scope14' : Unused code path elimination
 * Block '<Root>/Scope15' : Unused code path elimination
 * Block '<Root>/Scope2' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<Root>/Scope4' : Unused code path elimination
 * Block '<Root>/Scope5' : Unused code path elimination
 * Block '<Root>/Scope6' : Unused code path elimination
 * Block '<Root>/Scope7' : Unused code path elimination
 * Block '<Root>/Scope8' : Unused code path elimination
 * Block '<Root>/Scope9' : Unused code path elimination
 * Block '<S5>/Scope' : Unused code path elimination
 * Block '<S5>/Scope1' : Unused code path elimination
 * Block '<S107>/Data Type Duplicate' : Unused code path elimination
 * Block '<S107>/Data Type Propagation' : Unused code path elimination
 * Block '<S62>/Scope' : Unused code path elimination
 * Block '<S62>/Scope1' : Unused code path elimination
 * Block '<S5>/Servo Angle' : Unused code path elimination
 * Block '<S5>/To Workspace' : Unused code path elimination
 * Block '<S5>/To Workspace1' : Unused code path elimination
 * Block '<S5>/Valve Angle' : Unused code path elimination
 * Block '<S5>/Valve Fractional Area' : Unused code path elimination
 * Block '<S6>/Fluid Masses' : Unused code path elimination
 * Block '<S6>/N2 Densities' : Unused code path elimination
 * Block '<S6>/Nitrogen Flow Rate' : Unused code path elimination
 * Block '<S6>/Nitrogen Temp.' : Unused code path elimination
 * Block '<S6>/Scope' : Unused code path elimination
 * Block '<S6>/Scope1' : Unused code path elimination
 * Block '<S6>/Scope2' : Unused code path elimination
 * Block '<S6>/Water Flow Rate' : Unused code path elimination
 * Block '<Root>/Tank Pressures' : Unused code path elimination
 * Block '<Root>/Regulator Valve Kv' : Eliminated nontunable gain of 1
 * Block '<S2>/Output' : Eliminate redundant signal conversion block
 * Block '<S3>/Output' : Eliminate redundant signal conversion block
 * Block '<S4>/Output' : Eliminate redundant signal conversion block
 * Block '<S61>/Output' : Eliminate redundant signal conversion block
 * Block '<S6>/Gain2' : Eliminated nontunable gain of 1
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'EReg_Tank_Drain_discrete'
 * '<S1>'   : 'EReg_Tank_Drain_discrete/PID Controller'
 * '<S2>'   : 'EReg_Tank_Drain_discrete/Repeating Sequence'
 * '<S3>'   : 'EReg_Tank_Drain_discrete/Repeating Sequence1'
 * '<S4>'   : 'EReg_Tank_Drain_discrete/Repeating Sequence2'
 * '<S5>'   : 'EReg_Tank_Drain_discrete/Servo Actuated Valve'
 * '<S6>'   : 'EReg_Tank_Drain_discrete/Tank Plant Model'
 * '<S7>'   : 'EReg_Tank_Drain_discrete/PID Controller/Anti-windup'
 * '<S8>'   : 'EReg_Tank_Drain_discrete/PID Controller/D Gain'
 * '<S9>'   : 'EReg_Tank_Drain_discrete/PID Controller/External Derivative'
 * '<S10>'  : 'EReg_Tank_Drain_discrete/PID Controller/Filter'
 * '<S11>'  : 'EReg_Tank_Drain_discrete/PID Controller/Filter ICs'
 * '<S12>'  : 'EReg_Tank_Drain_discrete/PID Controller/I Gain'
 * '<S13>'  : 'EReg_Tank_Drain_discrete/PID Controller/Ideal P Gain'
 * '<S14>'  : 'EReg_Tank_Drain_discrete/PID Controller/Ideal P Gain Fdbk'
 * '<S15>'  : 'EReg_Tank_Drain_discrete/PID Controller/Integrator'
 * '<S16>'  : 'EReg_Tank_Drain_discrete/PID Controller/Integrator ICs'
 * '<S17>'  : 'EReg_Tank_Drain_discrete/PID Controller/N Copy'
 * '<S18>'  : 'EReg_Tank_Drain_discrete/PID Controller/N Gain'
 * '<S19>'  : 'EReg_Tank_Drain_discrete/PID Controller/P Copy'
 * '<S20>'  : 'EReg_Tank_Drain_discrete/PID Controller/Parallel P Gain'
 * '<S21>'  : 'EReg_Tank_Drain_discrete/PID Controller/Reset Signal'
 * '<S22>'  : 'EReg_Tank_Drain_discrete/PID Controller/Saturation'
 * '<S23>'  : 'EReg_Tank_Drain_discrete/PID Controller/Saturation Fdbk'
 * '<S24>'  : 'EReg_Tank_Drain_discrete/PID Controller/Sum'
 * '<S25>'  : 'EReg_Tank_Drain_discrete/PID Controller/Sum Fdbk'
 * '<S26>'  : 'EReg_Tank_Drain_discrete/PID Controller/Tracking Mode'
 * '<S27>'  : 'EReg_Tank_Drain_discrete/PID Controller/Tracking Mode Sum'
 * '<S28>'  : 'EReg_Tank_Drain_discrete/PID Controller/Tsamp - Integral'
 * '<S29>'  : 'EReg_Tank_Drain_discrete/PID Controller/Tsamp - Ngain'
 * '<S30>'  : 'EReg_Tank_Drain_discrete/PID Controller/postSat Signal'
 * '<S31>'  : 'EReg_Tank_Drain_discrete/PID Controller/preInt Signal'
 * '<S32>'  : 'EReg_Tank_Drain_discrete/PID Controller/preSat Signal'
 * '<S33>'  : 'EReg_Tank_Drain_discrete/PID Controller/Anti-windup/Passthrough'
 * '<S34>'  : 'EReg_Tank_Drain_discrete/PID Controller/D Gain/External Parameters'
 * '<S35>'  : 'EReg_Tank_Drain_discrete/PID Controller/External Derivative/Error'
 * '<S36>'  : 'EReg_Tank_Drain_discrete/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S37>'  : 'EReg_Tank_Drain_discrete/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S38>'  : 'EReg_Tank_Drain_discrete/PID Controller/I Gain/External Parameters'
 * '<S39>'  : 'EReg_Tank_Drain_discrete/PID Controller/Ideal P Gain/Passthrough'
 * '<S40>'  : 'EReg_Tank_Drain_discrete/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S41>'  : 'EReg_Tank_Drain_discrete/PID Controller/Integrator/Discrete'
 * '<S42>'  : 'EReg_Tank_Drain_discrete/PID Controller/Integrator ICs/Internal IC'
 * '<S43>'  : 'EReg_Tank_Drain_discrete/PID Controller/N Copy/Disabled'
 * '<S44>'  : 'EReg_Tank_Drain_discrete/PID Controller/N Gain/External Parameters'
 * '<S45>'  : 'EReg_Tank_Drain_discrete/PID Controller/P Copy/Disabled'
 * '<S46>'  : 'EReg_Tank_Drain_discrete/PID Controller/Parallel P Gain/External Parameters'
 * '<S47>'  : 'EReg_Tank_Drain_discrete/PID Controller/Reset Signal/Disabled'
 * '<S48>'  : 'EReg_Tank_Drain_discrete/PID Controller/Saturation/Passthrough'
 * '<S49>'  : 'EReg_Tank_Drain_discrete/PID Controller/Saturation Fdbk/Disabled'
 * '<S50>'  : 'EReg_Tank_Drain_discrete/PID Controller/Sum/Sum_PID'
 * '<S51>'  : 'EReg_Tank_Drain_discrete/PID Controller/Sum Fdbk/Disabled'
 * '<S52>'  : 'EReg_Tank_Drain_discrete/PID Controller/Tracking Mode/Disabled'
 * '<S53>'  : 'EReg_Tank_Drain_discrete/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S54>'  : 'EReg_Tank_Drain_discrete/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S55>'  : 'EReg_Tank_Drain_discrete/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S56>'  : 'EReg_Tank_Drain_discrete/PID Controller/postSat Signal/Forward_Path'
 * '<S57>'  : 'EReg_Tank_Drain_discrete/PID Controller/preInt Signal/Internal PreInt'
 * '<S58>'  : 'EReg_Tank_Drain_discrete/PID Controller/preSat Signal/Forward_Path'
 * '<S59>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/ Valve Gear Conversion'
 * '<S60>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/ Valve area'
 * '<S61>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Repeating Sequence2'
 * '<S62>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo'
 * '<S63>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller'
 * '<S64>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/Signal normalisation'
 * '<S65>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Anti-windup'
 * '<S66>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/D Gain'
 * '<S67>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/External Derivative'
 * '<S68>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Filter'
 * '<S69>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Filter ICs'
 * '<S70>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/I Gain'
 * '<S71>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Ideal P Gain'
 * '<S72>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Ideal P Gain Fdbk'
 * '<S73>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Integrator'
 * '<S74>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Integrator ICs'
 * '<S75>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/N Copy'
 * '<S76>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/N Gain'
 * '<S77>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/P Copy'
 * '<S78>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Parallel P Gain'
 * '<S79>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Reset Signal'
 * '<S80>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Saturation'
 * '<S81>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Saturation Fdbk'
 * '<S82>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Sum'
 * '<S83>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Sum Fdbk'
 * '<S84>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Tracking Mode'
 * '<S85>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Tracking Mode Sum'
 * '<S86>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Tsamp - Integral'
 * '<S87>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Tsamp - Ngain'
 * '<S88>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/postSat Signal'
 * '<S89>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/preInt Signal'
 * '<S90>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/preSat Signal'
 * '<S91>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Anti-windup/Passthrough'
 * '<S92>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/D Gain/Internal Parameters'
 * '<S93>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/External Derivative/Error'
 * '<S94>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S95>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S96>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/I Gain/Internal Parameters'
 * '<S97>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Ideal P Gain/Passthrough'
 * '<S98>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S99>'  : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Integrator/Discrete'
 * '<S100>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Integrator ICs/Internal IC'
 * '<S101>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/N Copy/Disabled'
 * '<S102>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/N Gain/Internal Parameters'
 * '<S103>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/P Copy/Disabled'
 * '<S104>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S105>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Reset Signal/Disabled'
 * '<S106>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Saturation/External'
 * '<S107>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Saturation/External/Saturation Dynamic'
 * '<S108>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Saturation Fdbk/Disabled'
 * '<S109>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Sum/Sum_PID'
 * '<S110>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Sum Fdbk/Disabled'
 * '<S111>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Tracking Mode/Disabled'
 * '<S112>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S113>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S114>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S115>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/postSat Signal/Forward_Path'
 * '<S116>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/preInt Signal/Internal PreInt'
 * '<S117>' : 'EReg_Tank_Drain_discrete/Servo Actuated Valve/Servo/PID Controller/preSat Signal/Forward_Path'
 * '<S118>' : 'EReg_Tank_Drain_discrete/Tank Plant Model/Component Flow Solver'
 * '<S119>' : 'EReg_Tank_Drain_discrete/Tank Plant Model/Gas Volume Properties'
 */
#endif                                 /* EReg_Tank_Drain_discrete_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
