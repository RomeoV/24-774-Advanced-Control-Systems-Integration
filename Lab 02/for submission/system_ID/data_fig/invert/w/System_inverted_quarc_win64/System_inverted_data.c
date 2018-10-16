/*
 * System_inverted_data.c
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

/* Block parameters (default storage) */
P_System_inverted_T System_inverted_P = {
  /* Variable: gain
   * Referenced by: '<Root>/Gain3'
   */
  0.001,

  /* Variable: lqr_k
   * Referenced by: '<Root>/Gain2'
   */
  { -7.0710678118655856, 48.0855164591744, -2.3960451168610408,
    4.0603029592695927 },

  /* Mask Parameter: BandLimitedWhiteNoise1_Cov
   * Referenced by: '<S2>/Output'
   */
  0.1,

  /* Mask Parameter: BandLimitedWhiteNoise1_seed
   * Referenced by: '<S2>/White Noise'
   */
  23341.0,

  /* Mask Parameter: HILReadEncoderTimebase_clock
   * Referenced by: '<S6>/HIL Read Encoder Timebase'
   */
  0,

  /* Mask Parameter: HILReadEncoderTimebase_channels
   * Referenced by: '<S6>/HIL Read Encoder Timebase'
   */
  { 0U, 1U },

  /* Mask Parameter: HILWriteAnalog_channels
   * Referenced by: '<S6>/HIL Write Analog'
   */
  0U,

  /* Mask Parameter: HILReadEncoderTimebase_samples_
   * Referenced by: '<S6>/HIL Read Encoder Timebase'
   */
  1000U,

  /* Expression: set_other_outputs_at_terminate
   * Referenced by: '<S6>/HIL Initialize'
   */
  1.0,

  /* Expression: set_other_outputs_at_switch_out
   * Referenced by: '<S6>/HIL Initialize'
   */
  0.0,

  /* Expression: set_other_outputs_at_start
   * Referenced by: '<S6>/HIL Initialize'
   */
  1.0,

  /* Expression: set_other_outputs_at_switch_in
   * Referenced by: '<S6>/HIL Initialize'
   */
  0.0,

  /* Expression: final_analog_outputs
   * Referenced by: '<S6>/HIL Initialize'
   */
  0.0,

  /* Expression: final_other_outputs
   * Referenced by: '<S6>/HIL Initialize'
   */
  { 1.0, 0.0, 0.0 },

  /* Expression: analog_input_maximums
   * Referenced by: '<S6>/HIL Initialize'
   */
  3.0,

  /* Expression: analog_input_minimums
   * Referenced by: '<S6>/HIL Initialize'
   */
  -3.0,

  /* Expression: analog_output_maximums
   * Referenced by: '<S6>/HIL Initialize'
   */
  15.0,

  /* Expression: analog_output_minimums
   * Referenced by: '<S6>/HIL Initialize'
   */
  -15.0,

  /* Expression: initial_analog_outputs
   * Referenced by: '<S6>/HIL Initialize'
   */
  0.0,

  /* Expression: watchdog_analog_outputs
   * Referenced by: '<S6>/HIL Initialize'
   */
  0.0,

  /* Expression: initial_other_outputs
   * Referenced by: '<S6>/HIL Initialize'
   */
  { 0.0, 1.0, 0.0 },

  /* Expression: watchdog_other_outputs
   * Referenced by: '<S6>/HIL Initialize'
   */
  { 0.0, 0.0, 1.0 },

  /* Expression: 2*pi/512/4
   * Referenced by: '<S13>/Pendulum:  counts to rad'
   */
  0.0030679615757712823,

  /* Expression: -2*pi/512/4
   * Referenced by: '<S13>/Arm: counts to rad'
   */
  -0.0030679615757712823,

  /* Expression: 0
   * Referenced by: '<Root>/Constant'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S2>/White Noise'
   */
  0.0,

  /* Computed Parameter: WhiteNoise_StdDev
   * Referenced by: '<S2>/White Noise'
   */
  1.0,

  /* Computed Parameter: Internal_A
   * Referenced by: '<S4>/Internal'
   */
  -50.0,

  /* Computed Parameter: Internal_B
   * Referenced by: '<S4>/Internal'
   */
  64.0,

  /* Computed Parameter: Internal_C
   * Referenced by: '<S4>/Internal'
   */
  -39.0625,

  /* Computed Parameter: Internal_D
   * Referenced by: '<S4>/Internal'
   */
  50.0,

  /* Expression: 0.0
   * Referenced by: '<S4>/Internal'
   */
  0.0,

  /* Computed Parameter: Internal_A_f
   * Referenced by: '<S5>/Internal'
   */
  -50.0,

  /* Computed Parameter: Internal_B_o
   * Referenced by: '<S5>/Internal'
   */
  64.0,

  /* Computed Parameter: Internal_C_k
   * Referenced by: '<S5>/Internal'
   */
  -39.0625,

  /* Computed Parameter: Internal_D_b
   * Referenced by: '<S5>/Internal'
   */
  50.0,

  /* Expression: 0.0
   * Referenced by: '<S5>/Internal'
   */
  0.0,

  /* Expression: 10
   * Referenced by: '<S6>/Saturation'
   */
  10.0,

  /* Expression: -10
   * Referenced by: '<S6>/Saturation'
   */
  -10.0,

  /* Expression: -1
   * Referenced by: '<S6>/For +ve CCW'
   */
  -1.0,

  /* Computed Parameter: HILInitialize_CKChannels
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_DOWatchdog
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_EIInitial
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_AIChannels
   * Referenced by: '<S6>/HIL Initialize'
   */
  0U,

  /* Computed Parameter: HILInitialize_AOChannels
   * Referenced by: '<S6>/HIL Initialize'
   */
  0U,

  /* Computed Parameter: HILInitialize_DOChannels
   * Referenced by: '<S6>/HIL Initialize'
   */
  0U,

  /* Computed Parameter: HILInitialize_EIChannels
   * Referenced by: '<S6>/HIL Initialize'
   */
  { 0U, 1U },

  /* Computed Parameter: HILInitialize_EIQuadrature
   * Referenced by: '<S6>/HIL Initialize'
   */
  4U,

  /* Computed Parameter: HILInitialize_OOChannels
   * Referenced by: '<S6>/HIL Initialize'
   */
  { 11000U, 11001U, 11002U },

  /* Computed Parameter: HILInitialize_Active
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_AOTerminate
   * Referenced by: '<S6>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILInitialize_AOExit
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_DOTerminate
   * Referenced by: '<S6>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILInitialize_DOExit
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POTerminate
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POExit
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_CKPStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_CKPEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_CKStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_CKEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_AIPStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_AIPEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_AOPStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_AOPEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_AOStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILInitialize_AOEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_AOReset
   * Referenced by: '<S6>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILInitialize_DOPStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_DOPEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_DOStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILInitialize_DOEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_DOReset
   * Referenced by: '<S6>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILInitialize_EIPStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_EIPEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_EIStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILInitialize_EIEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POPStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POPEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POStart
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POEnter
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_POReset
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_OOReset
   * Referenced by: '<S6>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILInitialize_DOFinal
   * Referenced by: '<S6>/HIL Initialize'
   */
  0,

  /* Computed Parameter: HILInitialize_DOInitial
   * Referenced by: '<S6>/HIL Initialize'
   */
  1,

  /* Computed Parameter: HILReadEncoderTimebase_Active
   * Referenced by: '<S6>/HIL Read Encoder Timebase'
   */
  1,

  /* Computed Parameter: HILWriteAnalog_Active
   * Referenced by: '<S6>/HIL Write Analog'
   */
  0
};
