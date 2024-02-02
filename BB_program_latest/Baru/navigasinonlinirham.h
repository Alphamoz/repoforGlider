//
// File: navigasinonlinirham.h
//
// Code generated for Simulink model 'navigasinonlinirham'.
//
// Model version                  : 1.24
// Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
// C/C++ source code generated on : Wed Sep 20 05:57:54 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_navigasinonlinirham_h_
#define RTW_HEADER_navigasinonlinirham_h_
#include <math.h>
#include <string.h>
#ifndef navigasinonlinirham_COMMON_INCLUDES_
# define navigasinonlinirham_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 // navigasinonlinirham_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

// Block signals and states (auto storage) for system '<Root>'
typedef struct {
  real_T P_k[25];                      // '<S5>/MATLAB Function'
  real_T xhat[5];                      // '<S5>/MATLAB Function'
  real_T A[25];                        // '<S5>/MATLAB Function'
  real_T C[15];                        // '<S5>/MATLAB Function'
  real_T R[9];                         // '<S5>/MATLAB Function'
  real_T Add;                          // '<S7>/Add'
  real_T lat;                          // '<S7>/MATLAB Function'
  real_T long_a;                       // '<S7>/MATLAB Function'
  real_T depth;                        // '<S7>/MATLAB Function'
  real_T aimuN;                        // '<S4>/MATLAB Function'
  real_T aimuE;                        // '<S4>/MATLAB Function'
  real_T aimuD;                        // '<S4>/MATLAB Function'
  real_T Memory1;                      // '<S13>/Memory1'
  real_T Memory;                       // '<S13>/Memory'
  real_T Sum;                          // '<S13>/Sum'
  real_T Q;                            // '<S5>/MATLAB Function'
  real_T radius;                       // '<S5>/MATLAB Function'
  real_T g;                            // '<S5>/MATLAB Function'
  real_T dt;                           // '<S5>/MATLAB Function'
  real_T Memory1_PreviousInput;        // '<S13>/Memory1'
  real_T Memory_PreviousInput;         // '<S13>/Memory'
  int8_T If_ActiveSubsystem;           // '<S3>/If'
  boolean_T P_not_empty;               // '<S5>/MATLAB Function'
} DW;

// Continuous states (auto storage)
typedef struct {
  real_T Integrator_CSTATE;            // '<S7>/Integrator'
  real_T Integrator1_CSTATE;           // '<S7>/Integrator1'
  real_T Integrator2_CSTATE;           // '<S7>/Integrator2'
  real_T Integrator_CSTATE_a;          // '<S4>/Integrator'
  real_T Integrator1_CSTATE_g;         // '<S4>/Integrator1'
  real_T Integrator2_CSTATE_o;         // '<S4>/Integrator2'
} X;

// State derivatives (auto storage)
typedef struct {
  real_T Integrator_CSTATE;            // '<S7>/Integrator'
  real_T Integrator1_CSTATE;           // '<S7>/Integrator1'
  real_T Integrator2_CSTATE;           // '<S7>/Integrator2'
  real_T Integrator_CSTATE_a;          // '<S4>/Integrator'
  real_T Integrator1_CSTATE_g;         // '<S4>/Integrator1'
  real_T Integrator2_CSTATE_o;         // '<S4>/Integrator2'
} XDot;

// State disabled
typedef struct {
  boolean_T Integrator_CSTATE;         // '<S7>/Integrator'
  boolean_T Integrator1_CSTATE;        // '<S7>/Integrator1'
  boolean_T Integrator2_CSTATE;        // '<S7>/Integrator2'
  boolean_T Integrator_CSTATE_a;       // '<S4>/Integrator'
  boolean_T Integrator1_CSTATE_g;      // '<S4>/Integrator1'
  boolean_T Integrator2_CSTATE_o;      // '<S4>/Integrator2'
} XDis;

#ifndef ODE3_INTG
#define ODE3_INTG

// ODE3 Integration Data
typedef struct {
  real_T *y;                           // output
  real_T *f[3];                        // derivatives
} ODE3_IntgData;

#endif

// External inputs (root inport signals with auto storage)
typedef struct {
  real_T Inudot;                       // '<Root>/Inudot'
  real_T Invdot;                       // '<Root>/Invdot'
  real_T Inwdot;                       // '<Root>/Inwdot'
  real_T Inteta;                       // '<Root>/Inteta'
  real_T Inyaw;                        // '<Root>/Inyaw'
  real_T Inphi;                        // '<Root>/Inphi'
  real_T Inu;                          // '<Root>/Inu'
  real_T Inv;                          // '<Root>/Inv'
  real_T Inw;                          // '<Root>/Inw'
  real_T Inz;                          // '<Root>/Inz'
  real_T Inpawalx;                     // '<Root>/Inpawalx'
  real_T Inpawaly;                     // '<Root>/Inpawaly'
  real_T Inpawalz;                     // '<Root>/Inpawalz'
} ExtU;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T Outlat;                       // '<Root>/Outlat'
  real_T Outlong;                      // '<Root>/Outlong'
  real_T Outz;                         // '<Root>/Outz'
  real_T Outposx;                      // '<Root>/Outposx'
  real_T Outposy;                      // '<Root>/Outposy'
  real_T Outposz;                      // '<Root>/Outposz'
  real_T Outkecx;                      // '<Root>/Outkecx '
  real_T Outkecy;                      // '<Root>/Outkecy'
  real_T Outkecz;                      // '<Root>/Outkecz'
} ExtY;

// Real-time Model Data Structure
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  //
  //  ModelData:
  //  The following substructure contains information regarding
  //  the data used in the model.

  struct {
    X *contStates;
    int_T *periodicContStateIndices;
    real_T *periodicContStateRanges;
    real_T *derivs;
    boolean_T *contStateDisabled;
    boolean_T zCCacheNeedsReset;
    boolean_T derivCacheNeedsReset;
    boolean_T blkStateChange;
    real_T odeY[6];
    real_T odeF[3][6];
    ODE3_IntgData intgData;
  } ModelData;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    uint16_T clockTick0;
    time_T stepSize0;
    uint16_T clockTick1;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model navigasinonlinirham
class navigasinonlinirhamModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  navigasinonlinirhamModelClass();

  // Destructor
  ~navigasinonlinirhamModelClass();

  // Real-Time Model get method
  RT_MODEL * getRTM();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;
  X rtX;                               // Block continuous states

  // Real-Time Model
  RT_MODEL rtM;

  // Continuous states update member function
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  // Derivatives member function
  void navigasinonlinirham_derivatives();
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S2>/Scope' : Unused code path elimination
//  Block '<S2>/Scope1' : Unused code path elimination
//  Block '<S2>/Scope2' : Unused code path elimination
//  Block '<S2>/Scope3' : Unused code path elimination
//  Block '<S2>/Scope4' : Unused code path elimination
//  Block '<S2>/Scope5' : Unused code path elimination
//  Block '<S2>/Scope6' : Unused code path elimination
//  Block '<S2>/Scope7' : Unused code path elimination
//  Block '<S5>/Scope' : Unused code path elimination
//  Block '<S6>/Scope' : Unused code path elimination
//  Block '<S6>/Scope1' : Unused code path elimination
//  Block '<S6>/Scope2' : Unused code path elimination
//  Block '<S6>/Scope3' : Unused code path elimination
//  Block '<S6>/Scope4' : Unused code path elimination
//  Block '<S6>/Scope5' : Unused code path elimination
//  Block '<S1>/Scope' : Unused code path elimination
//  Block '<S1>/Scope1' : Unused code path elimination
//  Block '<S1>/Scope2' : Unused code path elimination
//  Block '<S1>/Scope3' : Unused code path elimination
//  Block '<S8>/Scope' : Unused code path elimination
//  Block '<S8>/Scope1' : Unused code path elimination
//  Block '<S8>/Scope2' : Unused code path elimination
//  Block '<S10>/Scope' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'navigasinonlinirham'
//  '<S1>'   : 'navigasinonlinirham/Navigasi'
//  '<S2>'   : 'navigasinonlinirham/Navigasi/ Transformasi koordinat DVL'
//  '<S3>'   : 'navigasinonlinirham/Navigasi/Complementary Filter1'
//  '<S4>'   : 'navigasinonlinirham/Navigasi/IMU'
//  '<S5>'   : 'navigasinonlinirham/Navigasi/Kalman Filter'
//  '<S6>'   : 'navigasinonlinirham/Navigasi/Model DVL'
//  '<S7>'   : 'navigasinonlinirham/Navigasi/Perhitungan Posisi NED'
//  '<S8>'   : 'navigasinonlinirham/Navigasi/Transformasi koordinat IMU'
//  '<S9>'   : 'navigasinonlinirham/Navigasi/galat kecepatan'
//  '<S10>'  : 'navigasinonlinirham/Navigasi/kecepatan AUG'
//  '<S11>'  : 'navigasinonlinirham/Navigasi/ Transformasi koordinat DVL/MATLAB Function'
//  '<S12>'  : 'navigasinonlinirham/Navigasi/Complementary Filter1/If Action Subsystem'
//  '<S13>'  : 'navigasinonlinirham/Navigasi/Complementary Filter1/If Action Subsystem1'
//  '<S14>'  : 'navigasinonlinirham/Navigasi/IMU/MATLAB Function'
//  '<S15>'  : 'navigasinonlinirham/Navigasi/Kalman Filter/MATLAB Function'
//  '<S16>'  : 'navigasinonlinirham/Navigasi/Model DVL/MATLAB Function'
//  '<S17>'  : 'navigasinonlinirham/Navigasi/Model DVL/MATLAB Function1'
//  '<S18>'  : 'navigasinonlinirham/Navigasi/Model DVL/MATLAB Function2'
//  '<S19>'  : 'navigasinonlinirham/Navigasi/Perhitungan Posisi NED/MATLAB Function'
//  '<S20>'  : 'navigasinonlinirham/Navigasi/Transformasi koordinat IMU/MATLAB Function'

#endif                                 // RTW_HEADER_navigasinonlinirham_h_

//
// File trailer for generated code.
//
// [EOF]
//
