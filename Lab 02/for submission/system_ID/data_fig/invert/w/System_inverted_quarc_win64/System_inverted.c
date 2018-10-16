/*
 * System_inverted.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "System_inverted".
 *
 * Model version              : 1.514
 * Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
 * C source code generated on : Thu Oct 11 17:57:26 2018
 *
 * Target selection: quarc_win64.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "System_inverted.h"
#include "System_inverted_private.h"
#include "System_inverted_dt.h"

/* Block signals (default storage) */
B_System_inverted_T System_inverted_B;

/* Continuous states */
X_System_inverted_T System_inverted_X;

/* Block states (default storage) */
DW_System_inverted_T System_inverted_DW;

/* Real-time model */
RT_MODEL_System_inverted_T System_inverted_M_;
RT_MODEL_System_inverted_T *const System_inverted_M = &System_inverted_M_;

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
  int_T nXc = 2;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  System_inverted_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  System_inverted_output();
  System_inverted_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  System_inverted_output();
  System_inverted_derivatives();

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

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T lo;
  uint32_T hi;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T y;
  real_T sr;
  real_T si;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

/* Model output function */
void System_inverted_output(void)
{
  real_T x;
  real_T u1;
  real_T u2;
  if (rtmIsMajorTimeStep(System_inverted_M)) {
    /* set solver stop time */
    if (!(System_inverted_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&System_inverted_M->solverInfo,
                            ((System_inverted_M->Timing.clockTickH0 + 1) *
        System_inverted_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&System_inverted_M->solverInfo,
                            ((System_inverted_M->Timing.clockTick0 + 1) *
        System_inverted_M->Timing.stepSize0 +
        System_inverted_M->Timing.clockTickH0 *
        System_inverted_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(System_inverted_M)) {
    System_inverted_M->Timing.t[0] = rtsiGetT(&System_inverted_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(System_inverted_M)) {
    /* S-Function (hil_read_encoder_timebase_block): '<S6>/HIL Read Encoder Timebase' */

    /* S-Function Block: System_inverted/Plant/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
    {
      t_error result;
      result = hil_task_read_encoder
        (System_inverted_DW.HILReadEncoderTimebase_Task, 1,
         &System_inverted_DW.HILReadEncoderTimebase_Buffer[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
      } else {
        System_inverted_B.HILReadEncoderTimebase_o1 =
          System_inverted_DW.HILReadEncoderTimebase_Buffer[0];
        System_inverted_B.HILReadEncoderTimebase_o2 =
          System_inverted_DW.HILReadEncoderTimebase_Buffer[1];
      }
    }

    /* Gain: '<S13>/Pendulum:  counts to rad' */
    System_inverted_B.Pendulumcountstorad =
      System_inverted_P.Pendulumcountstorad_Gain *
      System_inverted_B.HILReadEncoderTimebase_o2;

    /* Gain: '<S13>/Arm: counts to rad' */
    System_inverted_B.Armcountstorad = System_inverted_P.Armcountstorad_Gain *
      System_inverted_B.HILReadEncoderTimebase_o1;

    /* RandomNumber: '<S2>/White Noise' */
    System_inverted_B.WhiteNoise = System_inverted_DW.NextOutput;

    /* Gain: '<S2>/Output' */
    x = System_inverted_P.BandLimitedWhiteNoise1_Cov;
    x = sqrt(x);
    x /= 0.031622776601683791;
    System_inverted_B.Output = x * System_inverted_B.WhiteNoise;

    /* Gain: '<Root>/Gain3' */
    System_inverted_B.Gain3 = System_inverted_P.gain * System_inverted_B.Output;

    /* Sum: '<Root>/Sum3' */
    System_inverted_B.Sum3 = System_inverted_B.Pendulumcountstorad +
      System_inverted_B.Gain3;
  }

  /* StateSpace: '<S4>/Internal' */
  System_inverted_B.Internal = 0.0;
  System_inverted_B.Internal += System_inverted_P.Internal_C *
    System_inverted_X.Internal_CSTATE;
  System_inverted_B.Internal += System_inverted_P.Internal_D *
    System_inverted_B.Armcountstorad;

  /* StateSpace: '<S5>/Internal' */
  System_inverted_B.Internal_h = 0.0;
  System_inverted_B.Internal_h += System_inverted_P.Internal_C_k *
    System_inverted_X.Internal_CSTATE_m;
  System_inverted_B.Internal_h += System_inverted_P.Internal_D_b *
    System_inverted_B.Sum3;

  /* Sum: '<Root>/Sum' incorporates:
   *  Constant: '<Root>/Constant'
   */
  System_inverted_B.Sum[0] = System_inverted_P.Constant_Value -
    System_inverted_B.Armcountstorad;
  System_inverted_B.Sum[1] = System_inverted_P.Constant_Value -
    System_inverted_B.Sum3;
  System_inverted_B.Sum[2] = System_inverted_P.Constant_Value -
    System_inverted_B.Internal;
  System_inverted_B.Sum[3] = System_inverted_P.Constant_Value -
    System_inverted_B.Internal_h;

  /* Gain: '<Root>/Gain2' */
  x = System_inverted_P.lqr_k[0] * System_inverted_B.Sum[0];
  x += System_inverted_P.lqr_k[1] * System_inverted_B.Sum[1];
  x += System_inverted_P.lqr_k[2] * System_inverted_B.Sum[2];
  x += System_inverted_P.lqr_k[3] * System_inverted_B.Sum[3];
  System_inverted_B.Gain2 = x;

  /* Sum: '<Root>/Sum1' */
  System_inverted_B.Sum1 = System_inverted_B.Gain2;

  /* Saturate: '<S6>/Saturation' */
  x = System_inverted_B.Sum1;
  u1 = System_inverted_P.Saturation_LowerSat;
  u2 = System_inverted_P.Saturation_UpperSat;
  if (x > u2) {
    System_inverted_B.Saturation = u2;
  } else if (x < u1) {
    System_inverted_B.Saturation = u1;
  } else {
    System_inverted_B.Saturation = x;
  }

  /* End of Saturate: '<S6>/Saturation' */

  /* Gain: '<S6>/For +ve CCW' */
  System_inverted_B.ForveCCW = System_inverted_P.ForveCCW_Gain *
    System_inverted_B.Saturation;
  if (rtmIsMajorTimeStep(System_inverted_M)) {
    /* S-Function (hil_write_analog_block): '<S6>/HIL Write Analog' */

    /* S-Function Block: System_inverted/Plant/HIL Write Analog (hil_write_analog_block) */
    {
      t_error result;
      result = hil_write_analog(System_inverted_DW.HILInitialize_Card,
        &System_inverted_P.HILWriteAnalog_channels, 1,
        &System_inverted_B.ForveCCW);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
      }
    }

    /* Sum: '<Root>/Sum2' */
    System_inverted_B.Sum2 = System_inverted_B.Armcountstorad;

    /* SignalConversion: '<Root>/TmpSignal ConversionAtTo WorkspaceInport1' */
    System_inverted_B.TmpSignalConversionAtToWorkspac[0] =
      System_inverted_B.Gain2;
    System_inverted_B.TmpSignalConversionAtToWorkspac[1] =
      System_inverted_B.Sum2;
    System_inverted_B.TmpSignalConversionAtToWorkspac[2] =
      System_inverted_B.Sum3;
  }
}

/* Model update function */
void System_inverted_update(void)
{
  if (rtmIsMajorTimeStep(System_inverted_M)) {
    /* Update for RandomNumber: '<S2>/White Noise' */
    System_inverted_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&System_inverted_DW.RandSeed) * System_inverted_P.WhiteNoise_StdDev +
      System_inverted_P.WhiteNoise_Mean;
  }

  if (rtmIsMajorTimeStep(System_inverted_M)) {
    rt_ertODEUpdateContinuousStates(&System_inverted_M->solverInfo);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++System_inverted_M->Timing.clockTick0)) {
    ++System_inverted_M->Timing.clockTickH0;
  }

  System_inverted_M->Timing.t[0] = rtsiGetSolverStopTime
    (&System_inverted_M->solverInfo);

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick1"
     * and "Timing.stepSize1". Size of "clockTick1" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++System_inverted_M->Timing.clockTick1)) {
      ++System_inverted_M->Timing.clockTickH1;
    }

    System_inverted_M->Timing.t[1] = System_inverted_M->Timing.clockTick1 *
      System_inverted_M->Timing.stepSize1 +
      System_inverted_M->Timing.clockTickH1 *
      System_inverted_M->Timing.stepSize1 * 4294967296.0;
  }
}

/* Derivatives for root system: '<Root>' */
void System_inverted_derivatives(void)
{
  XDot_System_inverted_T *_rtXdot;
  _rtXdot = ((XDot_System_inverted_T *) System_inverted_M->derivs);

  /* Derivatives for StateSpace: '<S4>/Internal' */
  _rtXdot->Internal_CSTATE = 0.0;
  _rtXdot->Internal_CSTATE += System_inverted_P.Internal_A *
    System_inverted_X.Internal_CSTATE;
  _rtXdot->Internal_CSTATE += System_inverted_P.Internal_B *
    System_inverted_B.Armcountstorad;

  /* Derivatives for StateSpace: '<S5>/Internal' */
  _rtXdot->Internal_CSTATE_m = 0.0;
  _rtXdot->Internal_CSTATE_m += System_inverted_P.Internal_A_f *
    System_inverted_X.Internal_CSTATE_m;
  _rtXdot->Internal_CSTATE_m += System_inverted_P.Internal_B_o *
    System_inverted_B.Sum3;
}

/* Model initialize function */
void System_inverted_initialize(void)
{
  /* Start for S-Function (hil_initialize_block): '<S6>/HIL Initialize' */

  /* S-Function Block: System_inverted/Plant/HIL Initialize (hil_initialize_block) */
  {
    t_int result;
    t_boolean is_switching;
    result = hil_open("qube_servo2_usb", "0",
                      &System_inverted_DW.HILInitialize_Card);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(System_inverted_M, _rt_error_message);
      return;
    }

    is_switching = false;
    result = hil_set_card_specific_options(System_inverted_DW.HILInitialize_Card,
      " ", 2);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(System_inverted_M, _rt_error_message);
      return;
    }

    result = hil_watchdog_clear(System_inverted_DW.HILInitialize_Card);
    if (result < 0 && result != -QERR_HIL_WATCHDOG_CLEAR) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(System_inverted_M, _rt_error_message);
      return;
    }

    if ((System_inverted_P.HILInitialize_AIPStart && !is_switching) ||
        (System_inverted_P.HILInitialize_AIPEnter && is_switching)) {
      result = hil_set_analog_input_ranges(System_inverted_DW.HILInitialize_Card,
        &System_inverted_P.HILInitialize_AIChannels, 1U,
        &System_inverted_P.HILInitialize_AILow,
        &System_inverted_P.HILInitialize_AIHigh);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }

    if ((System_inverted_P.HILInitialize_AOPStart && !is_switching) ||
        (System_inverted_P.HILInitialize_AOPEnter && is_switching)) {
      result = hil_set_analog_output_ranges
        (System_inverted_DW.HILInitialize_Card,
         &System_inverted_P.HILInitialize_AOChannels, 1U,
         &System_inverted_P.HILInitialize_AOLow,
         &System_inverted_P.HILInitialize_AOHigh);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }

    if ((System_inverted_P.HILInitialize_AOStart && !is_switching) ||
        (System_inverted_P.HILInitialize_AOEnter && is_switching)) {
      result = hil_write_analog(System_inverted_DW.HILInitialize_Card,
        &System_inverted_P.HILInitialize_AOChannels, 1U,
        &System_inverted_P.HILInitialize_AOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }

    if (System_inverted_P.HILInitialize_AOReset) {
      result = hil_watchdog_set_analog_expiration_state
        (System_inverted_DW.HILInitialize_Card,
         &System_inverted_P.HILInitialize_AOChannels, 1U,
         &System_inverted_P.HILInitialize_AOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }

    result = hil_set_digital_directions(System_inverted_DW.HILInitialize_Card,
      NULL, 0U, &System_inverted_P.HILInitialize_DOChannels, 1U);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(System_inverted_M, _rt_error_message);
      return;
    }

    if ((System_inverted_P.HILInitialize_DOStart && !is_switching) ||
        (System_inverted_P.HILInitialize_DOEnter && is_switching)) {
      result = hil_write_digital(System_inverted_DW.HILInitialize_Card,
        &System_inverted_P.HILInitialize_DOChannels, 1U, (t_boolean *)
        &System_inverted_P.HILInitialize_DOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }

    if (System_inverted_P.HILInitialize_DOReset) {
      result = hil_watchdog_set_digital_expiration_state
        (System_inverted_DW.HILInitialize_Card,
         &System_inverted_P.HILInitialize_DOChannels, 1U, (const t_digital_state
          *) &System_inverted_P.HILInitialize_DOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }

    if ((System_inverted_P.HILInitialize_EIPStart && !is_switching) ||
        (System_inverted_P.HILInitialize_EIPEnter && is_switching)) {
      System_inverted_DW.HILInitialize_QuadratureModes[0] =
        System_inverted_P.HILInitialize_EIQuadrature;
      System_inverted_DW.HILInitialize_QuadratureModes[1] =
        System_inverted_P.HILInitialize_EIQuadrature;
      result = hil_set_encoder_quadrature_mode
        (System_inverted_DW.HILInitialize_Card,
         System_inverted_P.HILInitialize_EIChannels, 2U,
         (t_encoder_quadrature_mode *)
         &System_inverted_DW.HILInitialize_QuadratureModes[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }

    if ((System_inverted_P.HILInitialize_EIStart && !is_switching) ||
        (System_inverted_P.HILInitialize_EIEnter && is_switching)) {
      System_inverted_DW.HILInitialize_InitialEICounts[0] =
        System_inverted_P.HILInitialize_EIInitial;
      System_inverted_DW.HILInitialize_InitialEICounts[1] =
        System_inverted_P.HILInitialize_EIInitial;
      result = hil_set_encoder_counts(System_inverted_DW.HILInitialize_Card,
        System_inverted_P.HILInitialize_EIChannels, 2U,
        &System_inverted_DW.HILInitialize_InitialEICounts[0]);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }

    if ((System_inverted_P.HILInitialize_OOStart && !is_switching) ||
        (System_inverted_P.HILInitialize_OOEnter && is_switching)) {
      result = hil_write_other(System_inverted_DW.HILInitialize_Card,
        System_inverted_P.HILInitialize_OOChannels, 3U,
        System_inverted_P.HILInitialize_OOInitial);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }

    if (System_inverted_P.HILInitialize_OOReset) {
      result = hil_watchdog_set_other_expiration_state
        (System_inverted_DW.HILInitialize_Card,
         System_inverted_P.HILInitialize_OOChannels, 3U,
         System_inverted_P.HILInitialize_OOWatchdog);
      if (result < 0) {
        msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
          (_rt_error_message));
        rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        return;
      }
    }
  }

  /* Start for S-Function (hil_read_encoder_timebase_block): '<S6>/HIL Read Encoder Timebase' */

  /* S-Function Block: System_inverted/Plant/HIL Read Encoder Timebase (hil_read_encoder_timebase_block) */
  {
    t_error result;
    result = hil_task_create_encoder_reader
      (System_inverted_DW.HILInitialize_Card,
       System_inverted_P.HILReadEncoderTimebase_samples_,
       System_inverted_P.HILReadEncoderTimebase_channels, 2,
       &System_inverted_DW.HILReadEncoderTimebase_Task);
    if (result < 0) {
      msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
        (_rt_error_message));
      rtmSetErrorStatus(System_inverted_M, _rt_error_message);
    }
  }

  {
    uint32_T tseed;
    int32_T r;
    int32_T t;
    real_T tmp;

    /* InitializeConditions for RandomNumber: '<S2>/White Noise' */
    tmp = floor(System_inverted_P.BandLimitedWhiteNoise1_seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? (uint32_T)-(int32_T)(uint32_T)-tmp : (uint32_T)tmp;
    r = (int32_T)(tseed >> 16U);
    t = (int32_T)(tseed & 32768U);
    tseed = ((((tseed - ((uint32_T)r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    System_inverted_DW.RandSeed = tseed;
    System_inverted_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&System_inverted_DW.RandSeed) * System_inverted_P.WhiteNoise_StdDev +
      System_inverted_P.WhiteNoise_Mean;

    /* End of InitializeConditions for RandomNumber: '<S2>/White Noise' */

    /* InitializeConditions for StateSpace: '<S4>/Internal' */
    System_inverted_X.Internal_CSTATE =
      System_inverted_P.Internal_InitialCondition;

    /* InitializeConditions for StateSpace: '<S5>/Internal' */
    System_inverted_X.Internal_CSTATE_m =
      System_inverted_P.Internal_InitialCondition_b;
  }
}

/* Model terminate function */
void System_inverted_terminate(void)
{
  /* Terminate for S-Function (hil_initialize_block): '<S6>/HIL Initialize' */

  /* S-Function Block: System_inverted/Plant/HIL Initialize (hil_initialize_block) */
  {
    t_boolean is_switching;
    t_int result;
    t_uint32 num_final_analog_outputs = 0;
    t_uint32 num_final_digital_outputs = 0;
    t_uint32 num_final_other_outputs = 0;
    hil_task_stop_all(System_inverted_DW.HILInitialize_Card);
    hil_monitor_stop_all(System_inverted_DW.HILInitialize_Card);
    is_switching = false;
    if ((System_inverted_P.HILInitialize_AOTerminate && !is_switching) ||
        (System_inverted_P.HILInitialize_AOExit && is_switching)) {
      num_final_analog_outputs = 1U;
    }

    if ((System_inverted_P.HILInitialize_DOTerminate && !is_switching) ||
        (System_inverted_P.HILInitialize_DOExit && is_switching)) {
      num_final_digital_outputs = 1U;
    }

    if ((System_inverted_P.HILInitialize_OOTerminate && !is_switching) ||
        (System_inverted_P.HILInitialize_OOExit && is_switching)) {
      num_final_other_outputs = 3U;
    }

    if (0
        || num_final_analog_outputs > 0
        || num_final_digital_outputs > 0
        || num_final_other_outputs > 0
        ) {
      /* Attempt to write the final outputs atomically (due to firmware issue in old Q2-USB). Otherwise write channels individually */
      result = hil_write(System_inverted_DW.HILInitialize_Card
                         , &System_inverted_P.HILInitialize_AOChannels,
                         num_final_analog_outputs
                         , NULL, 0
                         , &System_inverted_P.HILInitialize_DOChannels,
                         num_final_digital_outputs
                         , System_inverted_P.HILInitialize_OOChannels,
                         num_final_other_outputs
                         , &System_inverted_P.HILInitialize_AOFinal
                         , NULL
                         , (t_boolean *)
                         &System_inverted_P.HILInitialize_DOFinal
                         , System_inverted_P.HILInitialize_OOFinal
                         );
      if (result == -QERR_HIL_WRITE_NOT_SUPPORTED) {
        t_error local_result;
        result = 0;

        /* The hil_write operation is not supported by this card. Write final outputs for each channel type */
        if (num_final_analog_outputs > 0) {
          local_result = hil_write_analog(System_inverted_DW.HILInitialize_Card,
            &System_inverted_P.HILInitialize_AOChannels,
            num_final_analog_outputs, &System_inverted_P.HILInitialize_AOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_digital_outputs > 0) {
          local_result = hil_write_digital(System_inverted_DW.HILInitialize_Card,
            &System_inverted_P.HILInitialize_DOChannels,
            num_final_digital_outputs, (t_boolean *)
            &System_inverted_P.HILInitialize_DOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (num_final_other_outputs > 0) {
          local_result = hil_write_other(System_inverted_DW.HILInitialize_Card,
            System_inverted_P.HILInitialize_OOChannels, num_final_other_outputs,
            System_inverted_P.HILInitialize_OOFinal);
          if (local_result < 0) {
            result = local_result;
          }
        }

        if (result < 0) {
          msg_get_error_messageA(NULL, result, _rt_error_message, sizeof
            (_rt_error_message));
          rtmSetErrorStatus(System_inverted_M, _rt_error_message);
        }
      }
    }

    hil_task_delete_all(System_inverted_DW.HILInitialize_Card);
    hil_monitor_delete_all(System_inverted_DW.HILInitialize_Card);
    hil_close(System_inverted_DW.HILInitialize_Card);
    System_inverted_DW.HILInitialize_Card = NULL;
  }
}

/*========================================================================*
 * Start of Classic call interface                                        *
 *========================================================================*/

/* Solver interface called by GRT_Main */
#ifndef USE_GENERATED_SOLVER

void rt_ODECreateIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEDestroyIntegrationData(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

void rt_ODEUpdateContinuousStates(RTWSolverInfo *si)
{
  UNUSED_PARAMETER(si);
  return;
}                                      /* do nothing */

#endif

void MdlOutputs(int_T tid)
{
  System_inverted_output();
  UNUSED_PARAMETER(tid);
}

void MdlUpdate(int_T tid)
{
  System_inverted_update();
  UNUSED_PARAMETER(tid);
}

void MdlInitializeSizes(void)
{
}

void MdlInitializeSampleTimes(void)
{
}

void MdlInitialize(void)
{
}

void MdlStart(void)
{
  System_inverted_initialize();
}

void MdlTerminate(void)
{
  System_inverted_terminate();
}

/* Registration function */
RT_MODEL_System_inverted_T *System_inverted(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)System_inverted_M, 0,
                sizeof(RT_MODEL_System_inverted_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&System_inverted_M->solverInfo,
                          &System_inverted_M->Timing.simTimeStep);
    rtsiSetTPtr(&System_inverted_M->solverInfo, &rtmGetTPtr(System_inverted_M));
    rtsiSetStepSizePtr(&System_inverted_M->solverInfo,
                       &System_inverted_M->Timing.stepSize0);
    rtsiSetdXPtr(&System_inverted_M->solverInfo, &System_inverted_M->derivs);
    rtsiSetContStatesPtr(&System_inverted_M->solverInfo, (real_T **)
                         &System_inverted_M->contStates);
    rtsiSetNumContStatesPtr(&System_inverted_M->solverInfo,
      &System_inverted_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&System_inverted_M->solverInfo,
      &System_inverted_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&System_inverted_M->solverInfo,
      &System_inverted_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&System_inverted_M->solverInfo,
      &System_inverted_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&System_inverted_M->solverInfo, (&rtmGetErrorStatus
      (System_inverted_M)));
    rtsiSetRTModelPtr(&System_inverted_M->solverInfo, System_inverted_M);
  }

  rtsiSetSimTimeStep(&System_inverted_M->solverInfo, MAJOR_TIME_STEP);
  System_inverted_M->intgData.y = System_inverted_M->odeY;
  System_inverted_M->intgData.f[0] = System_inverted_M->odeF[0];
  System_inverted_M->intgData.f[1] = System_inverted_M->odeF[1];
  System_inverted_M->intgData.f[2] = System_inverted_M->odeF[2];
  System_inverted_M->contStates = ((real_T *) &System_inverted_X);
  rtsiSetSolverData(&System_inverted_M->solverInfo, (void *)
                    &System_inverted_M->intgData);
  rtsiSetSolverName(&System_inverted_M->solverInfo,"ode3");

  /* Initialize timing info */
  {
    int_T *mdlTsMap = System_inverted_M->Timing.sampleTimeTaskIDArray;
    mdlTsMap[0] = 0;
    mdlTsMap[1] = 1;
    System_inverted_M->Timing.sampleTimeTaskIDPtr = (&mdlTsMap[0]);
    System_inverted_M->Timing.sampleTimes =
      (&System_inverted_M->Timing.sampleTimesArray[0]);
    System_inverted_M->Timing.offsetTimes =
      (&System_inverted_M->Timing.offsetTimesArray[0]);

    /* task periods */
    System_inverted_M->Timing.sampleTimes[0] = (0.0);
    System_inverted_M->Timing.sampleTimes[1] = (0.001);

    /* task offsets */
    System_inverted_M->Timing.offsetTimes[0] = (0.0);
    System_inverted_M->Timing.offsetTimes[1] = (0.0);
  }

  rtmSetTPtr(System_inverted_M, &System_inverted_M->Timing.tArray[0]);

  {
    int_T *mdlSampleHits = System_inverted_M->Timing.sampleHitArray;
    mdlSampleHits[0] = 1;
    mdlSampleHits[1] = 1;
    System_inverted_M->Timing.sampleHits = (&mdlSampleHits[0]);
  }

  rtmSetTFinal(System_inverted_M, 30.0);
  System_inverted_M->Timing.stepSize0 = 0.001;
  System_inverted_M->Timing.stepSize1 = 0.001;

  /* External mode info */
  System_inverted_M->Sizes.checksums[0] = (1737082375U);
  System_inverted_M->Sizes.checksums[1] = (3670918618U);
  System_inverted_M->Sizes.checksums[2] = (380511293U);
  System_inverted_M->Sizes.checksums[3] = (4067965611U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[1];
    System_inverted_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(System_inverted_M->extModeInfo,
      &System_inverted_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(System_inverted_M->extModeInfo,
                        System_inverted_M->Sizes.checksums);
    rteiSetTPtr(System_inverted_M->extModeInfo, rtmGetTPtr(System_inverted_M));
  }

  System_inverted_M->solverInfoPtr = (&System_inverted_M->solverInfo);
  System_inverted_M->Timing.stepSize = (0.001);
  rtsiSetFixedStepSize(&System_inverted_M->solverInfo, 0.001);
  rtsiSetSolverMode(&System_inverted_M->solverInfo, SOLVER_MODE_SINGLETASKING);

  /* block I/O */
  System_inverted_M->blockIO = ((void *) &System_inverted_B);

  {
    System_inverted_B.HILReadEncoderTimebase_o1 = 0.0;
    System_inverted_B.HILReadEncoderTimebase_o2 = 0.0;
    System_inverted_B.Pendulumcountstorad = 0.0;
    System_inverted_B.Armcountstorad = 0.0;
    System_inverted_B.WhiteNoise = 0.0;
    System_inverted_B.Output = 0.0;
    System_inverted_B.Gain3 = 0.0;
    System_inverted_B.Sum3 = 0.0;
    System_inverted_B.Internal = 0.0;
    System_inverted_B.Internal_h = 0.0;
    System_inverted_B.Sum[0] = 0.0;
    System_inverted_B.Sum[1] = 0.0;
    System_inverted_B.Sum[2] = 0.0;
    System_inverted_B.Sum[3] = 0.0;
    System_inverted_B.Gain2 = 0.0;
    System_inverted_B.Sum1 = 0.0;
    System_inverted_B.Saturation = 0.0;
    System_inverted_B.ForveCCW = 0.0;
    System_inverted_B.Sum2 = 0.0;
    System_inverted_B.TmpSignalConversionAtToWorkspac[0] = 0.0;
    System_inverted_B.TmpSignalConversionAtToWorkspac[1] = 0.0;
    System_inverted_B.TmpSignalConversionAtToWorkspac[2] = 0.0;
  }

  /* parameters */
  System_inverted_M->defaultParam = ((real_T *)&System_inverted_P);

  /* states (continuous) */
  {
    real_T *x = (real_T *) &System_inverted_X;
    System_inverted_M->contStates = (x);
    (void) memset((void *)&System_inverted_X, 0,
                  sizeof(X_System_inverted_T));
  }

  /* states (dwork) */
  System_inverted_M->dwork = ((void *) &System_inverted_DW);
  (void) memset((void *)&System_inverted_DW, 0,
                sizeof(DW_System_inverted_T));
  System_inverted_DW.HILInitialize_FilterFrequency[0] = 0.0;
  System_inverted_DW.HILInitialize_FilterFrequency[1] = 0.0;
  System_inverted_DW.NextOutput = 0.0;

  /* data type transition information */
  {
    static DataTypeTransInfo dtInfo;
    (void) memset((char_T *) &dtInfo, 0,
                  sizeof(dtInfo));
    System_inverted_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 16;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    /* Block I/O transition table */
    dtInfo.BTransTable = &rtBTransTable;

    /* Parameters transition table */
    dtInfo.PTransTable = &rtPTransTable;
  }

  /* Initialize Sizes */
  System_inverted_M->Sizes.numContStates = (2);/* Number of continuous states */
  System_inverted_M->Sizes.numPeriodicContStates = (0);/* Number of periodic continuous states */
  System_inverted_M->Sizes.numY = (0); /* Number of model outputs */
  System_inverted_M->Sizes.numU = (0); /* Number of model inputs */
  System_inverted_M->Sizes.sysDirFeedThru = (0);/* The model is not direct feedthrough */
  System_inverted_M->Sizes.numSampTimes = (2);/* Number of sample times */
  System_inverted_M->Sizes.numBlocks = (20);/* Number of blocks */
  System_inverted_M->Sizes.numBlockIO = (17);/* Number of block outputs */
  System_inverted_M->Sizes.numBlockPrms = (99);/* Sum of parameter "widths" */
  return System_inverted_M;
}

/*========================================================================*
 * End of Classic call interface                                          *
 *========================================================================*/
